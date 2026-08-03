// =============================================================================
//  wolf_position_node.cpp
//  WoLF-EKF-CA position solver (Duran-Martin et al. ICML 2024).
//
//  Pipeline:
//    aggregate 4 ToA → apply bias_values_ns → compute TDoA → EKF predict
//      → IMQ weighting (Eq. 17 paper) → weighted-likelihood update
//      → publish raw position to /state/position
//
//  Differences from position_solver_node:
//    - State: 9-D constant-acceleration [p(3), v(3), a(3)]
//      (vs 6-D constant-velocity in pipeline lama)
//    - TDoA enters EKF directly via h(p) = ||p-aᵢ|| - ||p-a_ref||
//      (Chan only used ONCE for initialization seed, not per blink)
//    - No Layer 1 (Predictive TDoA Gate) or Layer 2 (Student-t α-scaling).
//      WoLF IMQ weighting handles outliers via Generalised Bayes update
//      (Proposition 3.1, Theorem 3.2 — provably bounded PIF).
//    - sigma_a is adaptive: σ_a_eff = σ_a_min + gain·‖v‖
//      Small process noise when stationary → tight steady-state precision.
//      Larger when moving → responsive to maneuver.
//
//  Initialization (v0.2.0):
//    WoLF initializes immediately at anchor_config arrival, using pos_nominal_
//    as seed with a large initial covariance P0. EKF then converges to the
//    true position over the first few blinks. No "Chan must succeed N times"
//    gating — that approach (v0.1.0) deadlocked when Chan failed to converge
//    from a far-from-truth seed (last_pos_chan_ = pos_nominal_).
//
//  Changelog:
//    2026-05-31 v0.3.1 — koreksi default gate ZUPT (tuning, bukan perubahan
//                        logika): v_enter 0.08→0.30, v_exit 0.15→0.45,
//                        spread_enter 0.05→0.08. Jitter menggelembungkan ‖v̂‖
//                        EKF (~0.27 m/s saat fisik diam) → ambang awal terlalu
//                        ketat, HOLD tak pernah trigger. Terverifikasi hardware
//                        vs OptiTrack: jitter hover 51→24mm (−53%), HF jitter
//                        10.2→3.4mm, autokorelasi 0.85→0.92 (tanda EMA aktif).
//    2026-05-21 v0.3.0 — tambah ZUPT (Zero-Velocity Hold) post-filter pada
//                        output posisi. Saat drone diam (gate ‖v̂‖ state EKF +
//                        spread jendela + histeresis), output di-smooth keras
//                        (EMA cutoff rendah / median) untuk tekan jitter hover
//                        ~38mm → <15mm. Saat bergerak: passthrough. State EKF
//                        TIDAK disentuh (wrap murni). zupt_enabled=false =
//                        perilaku identik v0.2.0. Gate ‖v̂‖ aman terhadap
//                        ledakan (saat over-ekstrapolasi ‖v̂‖ tinggi → ZUPT off).
//    2026-05-21 v0.2.0 — fix init deadlock: WoLF now initializes from
//                        pos_nominal_ at anchor_config receipt with large P0.
//                        Removed dependency on Chan validity for init. Chan
//                        still computed per-blink for diagnostic publish on
//                        /state/position_chan, but its result is no longer
//                        required for WoLF operation.
//    2026-05-21 v0.1.0 — initial release. WoLF-EKF-CA with IMQ weighting,
//                        adaptive sigma_a, Chan median init from 50 blinks.
// =============================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "ips_msgs/msg/corrected_to_a.hpp"

#include "ips_nodes_cpp/chan_solver.hpp"

namespace ips {

using std::placeholders::_1;

static constexpr double C_LIGHT_M_S = 299792458.0;

static rclcpp::QoS qos_state_reliable() {
    rclcpp::QoS q(20);
    q.reliable().keep_last(20);
    return q;
}
static rclcpp::QoS qos_latched_config() {
    rclcpp::QoS q(1);
    q.reliable().transient_local().keep_last(1);
    return q;
}

// =============================================================================
//  WoLF-EKF-CA core (no ROS deps — easy to test offline)
// =============================================================================
class WolfEkfCa {
public:
    struct Config {
        std::array<Eigen::Vector3d, 4> anchors_ordered;  // ref = index 0
        double sigma_tdoa     = 0.10;   // R = σ²·I
        double sigma_a_min    = 0.05;   // process noise floor
        double sigma_a_gain   = 1.0;    // σ_a_eff = σ_a_min + gain·‖v‖
        double sigma_a_max    = 3.0;    // CAP σ_a_eff — cegah runaway saat manuver
        double wolf_c         = 0.3;    // IMQ soft threshold (m)
        double pos_std0       = 0.5;    // initial P diagonal — position
        double vel_std0       = 0.5;    // initial P diagonal — velocity
        double acc_std0       = 0.5;    // initial P diagonal — acceleration
    };

    explicit WolfEkfCa(const Config& cfg)
        : cfg_(cfg),
          x_(Eigen::Matrix<double, 9, 1>::Zero()),
          P_(Eigen::Matrix<double, 9, 9>::Identity())
    {
        // R = σ_tdoa² · I_3
        R_ = (cfg_.sigma_tdoa * cfg_.sigma_tdoa) *
             Eigen::Matrix3d::Identity();
    }

    void initialize(const Eigen::Vector3d& p0) {
        x_.setZero();
        x_.segment<3>(0) = p0;
        P_.setIdentity();
        P_(0, 0) = P_(1, 1) = P_(2, 2) = cfg_.pos_std0 * cfg_.pos_std0;
        P_(3, 3) = P_(4, 4) = P_(5, 5) = cfg_.vel_std0 * cfg_.vel_std0;
        P_(6, 6) = P_(7, 7) = P_(8, 8) = cfg_.acc_std0 * cfg_.acc_std0;
        initialized_ = true;
        last_weight_ = 1.0;
    }

    bool initialized() const { return initialized_; }
    const Eigen::Matrix<double, 9, 1>& state() const { return x_; }
    Eigen::Vector3d position() const { return x_.segment<3>(0); }
    Eigen::Vector3d velocity() const { return x_.segment<3>(3); }
    const Eigen::Matrix<double, 9, 9>& covariance() const { return P_; }
    double last_weight() const { return last_weight_; }

    // Deteksi divergensi: state tak finite (NaN/inf) atau posisi absurd jauh
    // di luar batas wajar. Dipakai untuk re-init dari Chan (cegah satu ledakan
    // merusak sisa sesi). pos_limit = jangkauan wajar dari origin (meter).
    bool is_diverged(double pos_limit) const {
        if (!x_.allFinite() || !P_.allFinite()) return true;
        const double r = x_.segment<3>(0).norm();
        if (r > pos_limit) return true;
        return false;
    }

    // Predict step (CA model): x_{t|t-1} = F·x_{t-1}, P = F·P·F' + Q
    void predict(double dt) {
        if (dt <= 0.0 || dt > 1.0) return;

        // F: 9x9 constant-acceleration
        Eigen::Matrix<double, 9, 9> F =
            Eigen::Matrix<double, 9, 9>::Identity();
        F.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
        F.block<3, 3>(0, 6) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
        F.block<3, 3>(3, 6) = dt * Eigen::Matrix3d::Identity();

        // Adaptive sigma_a: σ_a_eff = σ_a_min + gain · ‖v‖
        // CLAMP ke sigma_a_max: tanpa batas, ‖v‖ besar saat manuver → sa_eff
        // besar → q=sa_eff² → Q meledak → P meledak → ekstrapolasi → ‖v‖ naik
        // lagi (umpan balik kuadratik). Clamp memutus loop ini. (v0.3.3)
        const double v_norm = x_.segment<3>(3).norm();
        double sa_eff = cfg_.sigma_a_min + cfg_.sigma_a_gain * v_norm;
        if (cfg_.sigma_a_max > 0.0 && sa_eff > cfg_.sigma_a_max)
            sa_eff = cfg_.sigma_a_max;
        const double q      = sa_eff * sa_eff;

        // Q: white-noise jerk model, 9x9
        const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt,
                     dt5 = dt4 * dt;
        Eigen::Matrix<double, 9, 9> Q =
            Eigen::Matrix<double, 9, 9>::Zero();
        Q.block<3, 3>(0, 0) = (dt5 / 20.0) * q * I3;
        Q.block<3, 3>(0, 3) = (dt4 /  8.0) * q * I3;
        Q.block<3, 3>(0, 6) = (dt3 /  6.0) * q * I3;
        Q.block<3, 3>(3, 0) = Q.block<3, 3>(0, 3);
        Q.block<3, 3>(3, 3) = (dt3 /  3.0) * q * I3;
        Q.block<3, 3>(3, 6) = (dt2 /  2.0) * q * I3;
        Q.block<3, 3>(6, 0) = Q.block<3, 3>(0, 6);
        Q.block<3, 3>(6, 3) = Q.block<3, 3>(3, 6);
        Q.block<3, 3>(6, 6) = dt        * q * I3;

        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    // WoLF update step. measurement: TDoA[i] = (||p-aᵢ|| - ||p-a_ref||) for
    // i ∈ {1, 2, 3} (relative to anchors_ordered[0] as reference).
    // Returns IMQ weight (1.0 = inlier, →0 = strong outlier).
    double update(const Eigen::Vector3d& tdoa_meas) {
        const Eigen::Vector3d p   = x_.segment<3>(0);
        const Eigen::Vector3d aref = cfg_.anchors_ordered[0];
        const Eigen::Vector3d dref = p - aref;
        const double          nref = dref.norm() + 1e-12;
        const Eigen::Vector3d uref = dref / nref;

        // Predicted TDoA + Jacobian H (3x9): only position columns nonzero
        Eigen::Vector3d                      y_pred;
        Eigen::Matrix<double, 3, 9> H =
            Eigen::Matrix<double, 3, 9>::Zero();
        for (int k = 0; k < 3; ++k) {
            const Eigen::Vector3d d_i = p - cfg_.anchors_ordered[k + 1];
            const double          n_i = d_i.norm() + 1e-12;
            const Eigen::Vector3d u_i = d_i / n_i;
            y_pred(k) = n_i - nref;
            H.block<1, 3>(k, 0) = (u_i - uref).transpose();
        }

        const Eigen::Vector3d innov = tdoa_meas - y_pred;

        // IMQ weight (Eq. 17): w = (1 + ‖innov‖²/c²)^(-1/2)
        const double innov_sq = innov.squaredNorm();
        const double w        = 1.0 / std::sqrt(
            1.0 + innov_sq / (cfg_.wolf_c * cfg_.wolf_c));
        const double w2 = w * w;

        // WoLF update (Proposition 3.1):
        //   identical to KF update but R⁻¹ → w²·R⁻¹
        //   equivalent form: S = H·P·H' + R/w²
        const Eigen::Matrix3d R_eff = R_ / w2;
        const Eigen::Matrix3d S     = H * P_ * H.transpose() + R_eff;
        const Eigen::Matrix<double, 9, 3> K =
            P_ * H.transpose() * S.inverse();

        x_ = x_ + K * innov;

        // Joseph form for numerical stability
        const Eigen::Matrix<double, 9, 9> I9 =
            Eigen::Matrix<double, 9, 9>::Identity();
        const Eigen::Matrix<double, 9, 9> IKH = I9 - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R_eff * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        last_weight_ = w;
        return w;
    }

private:
    Config                                  cfg_;
    Eigen::Matrix<double, 9, 1>             x_;
    Eigen::Matrix<double, 9, 9>             P_;
    Eigen::Matrix3d                         R_;
    bool                                    initialized_ = false;
    double                                  last_weight_ = 1.0;
};

// =============================================================================
//  ZuptHold — Zero-Velocity Hold post-filter (v0.3.0)
//
//  WRAP, bukan rewrite. Beroperasi HANYA pada nilai posisi yang dikeluarkan
//  EKF; TIDAK menyentuh state internal EKF (predict/update/IMQ tetap jalan
//  normal). Tujuan: tekan jitter hover (~38mm) ke <15mm saat drone diam,
//  passthrough penuh saat bergerak.
//
//  Detektor diam (dua syarat + histeresis):
//    1. ‖v̂‖ dari STATE EKF (bukan turunan measurement) — aman terhadap
//       ledakan: saat model CA over-ekstrapolasi, ‖v̂‖ EKF TINGGI walau fisik
//       diam, sehingga ZUPT otomatis TIDAK aktif (tidak mengunci posisi salah).
//    2. spread = RMS std posisi jendela (√(σx²+σy²+σz²)) < ambang.
//  Histeresis: masuk HOLD bila kedua syarat terpenuhi N_ENTER sampel berturut;
//  keluar HOLD bila ‖v̂‖ > V_EXIT (responsif, V_EXIT > V_ENTER).
//
//  Saat HOLD: output = EMA cutoff-rendah (bukan hard-freeze) → konvergen ke
//  rata-rata DC, menumpas wander. Blend ramp masuk/keluar untuk transisi mulus.
//
//  EMA per-sampel pakai alpha = 1 − exp(−2π·f_c·dt) dengan dt AKTUAL tiap blink
//  (rate bervariasi ~43-48ms), sehingga cutoff konsisten terlepas jitter rate.
// =============================================================================
class ZuptHold {
public:
    struct Config {
        bool        enabled       = false;  // false = passthrough = perilaku lama
        double      v_enter       = 0.30;   // m/s; di bawah ini kandidat diam (v0.3.1)
        double      v_exit        = 0.45;   // m/s; histeresis keluar (v0.3.1)
        double      spread_enter  = 0.08;   // m; RMS std jendela utk konfirmasi diam (v0.3.1)
        double      window_s      = 1.0;    // s; panjang jendela deteksi spread
        int         n_enter       = 10;     // sampel berturut sebelum HOLD aktif
        double      hold_cutoff_hz= 0.2;    // Hz; cutoff EMA saat HOLD
        double      ramp_s        = 0.3;    // s; durasi blend masuk/keluar
        bool        use_median    = false;  // true = median jendela; false = EMA
    };

    explicit ZuptHold(const Config& cfg) : cfg_(cfg) {}

    void set_config(const Config& cfg) { cfg_ = cfg; }
    bool holding() const { return mode_hold_; }

    // Proses satu sampel posisi. p_ekf = posisi mentah dari EKF, v_norm =
    // ‖v̂‖ dari state EKF, dt = interval sejak blink sebelumnya (detik).
    // Return: posisi yang harus dipublikasikan.
    Eigen::Vector3d process(const Eigen::Vector3d& p_ekf,
                            double v_norm, double dt) {
        if (!cfg_.enabled) {
            // Passthrough penuh — identik perilaku pra-v0.3.0.
            return p_ekf;
        }

        // --- Update ring buffer posisi (untuk spread + median) ---
        push_buffer(p_ekf, dt);

        // --- Hitung spread (RMS std per-sumbu) atas jendela ---
        const double spread = compute_spread();

        // --- Mesin keadaan diam dengan histeresis ---
        const bool cand_still = (v_norm < cfg_.v_enter) &&
                                (spread < cfg_.spread_enter) &&
                                buffer_full_enough();
        if (!mode_hold_) {
            if (cand_still) {
                still_count_++;
                if (still_count_ >= cfg_.n_enter) {
                    // Masuk HOLD. Inisialisasi EMA dari posisi EKF saat ini,
                    // mulai ramp blend.
                    mode_hold_  = true;
                    ema_        = p_ekf;
                    blend_      = 0.0;     // 0 = full EKF, 1 = full HOLD
                }
            } else {
                still_count_ = 0;
            }
        } else {
            // Dalam HOLD; keluar bila kecepatan melewati ambang exit.
            if (v_norm > cfg_.v_exit) {
                mode_hold_   = false;
                still_count_ = 0;
                blend_out_   = 1.0;        // mulai ramp keluar dari HOLD→EKF
                last_hold_   = ema_;       // simpan posisi HOLD terakhir
            }
        }

        // --- Hitung target HOLD (EMA cutoff rendah atau median) ---
        if (mode_hold_) {
            if (cfg_.use_median) {
                ema_ = compute_median();
            } else {
                const double fc = cfg_.hold_cutoff_hz;
                double alpha = 1.0 - std::exp(-2.0 * M_PI * fc * dt);
                if (alpha < 0.0) alpha = 0.0;
                if (alpha > 1.0) alpha = 1.0;
                ema_ = ema_ + alpha * (p_ekf - ema_);
            }
        }

        // --- Blending untuk transisi mulus ---
        const double ramp_step = (cfg_.ramp_s > 1e-6) ? (dt / cfg_.ramp_s) : 1.0;

        if (mode_hold_) {
            // Ramp masuk: blend_ 0→1 (EKF→HOLD)
            blend_ += ramp_step;
            if (blend_ > 1.0) blend_ = 1.0;
            return (1.0 - blend_) * p_ekf + blend_ * ema_;
        }

        // Tidak HOLD. Kalau sedang ramp keluar, blend dari HOLD terakhir→EKF.
        if (blend_out_ > 0.0) {
            blend_out_ -= ramp_step;
            if (blend_out_ < 0.0) blend_out_ = 0.0;
            return blend_out_ * last_hold_ + (1.0 - blend_out_) * p_ekf;
        }

        // Passthrough (bergerak, tidak ada ramp aktif)
        return p_ekf;
    }

private:
    void push_buffer(const Eigen::Vector3d& p, double dt) {
        buf_pos_.push_back(p);
        buf_dt_.push_back(dt > 0.0 ? dt : 0.05);
        // Pangkas buffer agar total durasi ≈ window_s
        double total = 0.0;
        for (double d : buf_dt_) total += d;
        while (buf_pos_.size() > 1 && total > cfg_.window_s) {
            total -= buf_dt_.front();
            buf_dt_.erase(buf_dt_.begin());
            buf_pos_.erase(buf_pos_.begin());
        }
    }

    bool buffer_full_enough() const {
        // Minimal ~50% jendela terisi sebelum spread dianggap valid.
        double total = 0.0;
        for (double d : buf_dt_) total += d;
        return total >= 0.5 * cfg_.window_s;
    }

    double compute_spread() const {
        const std::size_t n = buf_pos_.size();
        if (n < 2) return 1e9;  // belum cukup data → jangan anggap diam
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto& p : buf_pos_) mean += p;
        mean /= static_cast<double>(n);
        Eigen::Vector3d var = Eigen::Vector3d::Zero();
        for (const auto& p : buf_pos_) {
            const Eigen::Vector3d d = p - mean;
            var += d.cwiseProduct(d);
        }
        var /= static_cast<double>(n);
        // RMS std = sqrt(σx² + σy² + σz²)
        return std::sqrt(var.sum());
    }

    Eigen::Vector3d compute_median() const {
        const std::size_t n = buf_pos_.size();
        if (n == 0) return Eigen::Vector3d::Zero();
        Eigen::Vector3d med;
        for (int k = 0; k < 3; ++k) {
            std::vector<double> v;
            v.reserve(n);
            for (const auto& p : buf_pos_) v.push_back(p(k));
            std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
            med(k) = v[v.size() / 2];
        }
        return med;
    }

    Config                          cfg_;
    std::vector<Eigen::Vector3d>    buf_pos_;
    std::vector<double>             buf_dt_;
    bool                            mode_hold_   = false;
    int                             still_count_ = 0;
    Eigen::Vector3d                 ema_         = Eigen::Vector3d::Zero();
    Eigen::Vector3d                 last_hold_   = Eigen::Vector3d::Zero();
    double                          blend_       = 0.0;  // ramp masuk (0→1)
    double                          blend_out_   = 0.0;  // ramp keluar (1→0)
};

// =============================================================================
//  ROS node wrapper
// =============================================================================
class WolfPositionNode : public rclcpp::Node {
public:
    WolfPositionNode() : rclcpp::Node("wolf_position") {
        // ===== Parameters (mirror position_solver where applicable) =====
        declare_parameter<std::string>("frame_id", "world");
        declare_parameter<int>        ("blink_buffer_size", 64);
        declare_parameter<double>     ("blink_publish_timeout_s", 0.5);
        declare_parameter<std::vector<int64_t>>(
            "rx_anchors_ordered", std::vector<int64_t>{2, 3, 4, 5});

        // antenna bias (same handling as pipeline lama)
        declare_parameter<std::vector<int64_t>>(
            "bias_anchor_ids", std::vector<int64_t>{2, 3, 4, 5});
        declare_parameter<std::vector<double>>(
            "bias_values_ns",
            std::vector<double>{0.0, 0.0, 0.0, 0.0});

        // WoLF / EKF-CA parameters
        declare_parameter<double>("wolf_sigma_tdoa",    0.10);
        declare_parameter<double>("wolf_sigma_a_min",   0.05);
        declare_parameter<double>("wolf_sigma_a_gain",  1.0);
        declare_parameter<double>("wolf_sigma_a_max",   3.0);
        declare_parameter<double>("wolf_c",             0.3);
        // v0.2.0: init langsung dari pos_nominal_ saat anchor_config datang.
        //   pos_std0 dibuat besar (default 1.5m) sehingga EKF dapat menarik
        //   state ke posisi tag sebenarnya walau init kasar.
        //   vel_std0/acc_std0 kecil (default 0.1) asumsi takeoff dari diam.
        declare_parameter<double>("wolf_pos_std0",      1.5);
        declare_parameter<double>("wolf_vel_std0",      0.1);
        declare_parameter<double>("wolf_acc_std0",      0.1);
        declare_parameter<double>("wolf_log_every_s",   5.0);

        declare_parameter<std::vector<double>>(
            "pos_nominal", std::vector<double>{1.0, 1.0, 1.75});
        declare_parameter<std::vector<double>>(
            "room_dim",    std::vector<double>{2.0, 2.0, 3.5});
        declare_parameter<double>("dt_nominal_s", 0.20);

        // ===== Zero-Velocity Hold (ZUPT) parameters (v0.3.0; gate v0.3.1) =====
        // Default gate dikoreksi setelah uji hardware: jitter menggelembungkan
        // ‖v̂‖ EKF (~0.27 m/s saat fisik diam), jadi ambang awal (0.08/0.15/0.05)
        // terlalu ketat. Nilai di bawah terbukti bekerja (jitter hover 51→24mm).
        declare_parameter<bool>  ("zupt_enabled",        false);
        declare_parameter<double>("zupt_v_enter",        0.30);
        declare_parameter<double>("zupt_v_exit",         0.45);
        declare_parameter<double>("zupt_spread_enter",   0.08);
        declare_parameter<double>("zupt_window_s",       1.0);
        declare_parameter<int>   ("zupt_n_enter",        10);
        declare_parameter<std::string>("zupt_hold_mode", "ema");
        declare_parameter<double>("zupt_hold_cutoff_hz", 0.2);
        declare_parameter<double>("zupt_ramp_s",         0.3);
        declare_parameter<double>("zupt_log_every_s",    5.0);

        // ===== Read parameters =====
        frame_id_  = get_parameter("frame_id").as_string();
        buf_size_  = static_cast<std::size_t>(
                        get_parameter("blink_buffer_size").as_int());
        timeout_s_ = get_parameter("blink_publish_timeout_s").as_double();

        auto rx_v  = get_parameter("rx_anchors_ordered").as_integer_array();
        if (rx_v.size() != 4) {
            RCLCPP_FATAL(get_logger(),
                "rx_anchors_ordered must have exactly 4 entries (got %zu)",
                rx_v.size());
            throw std::runtime_error("bad rx_anchors_ordered");
        }
        for (int i = 0; i < 4; ++i) rx_ids_[i] = static_cast<int>(rx_v[i]);

        auto bias_ids = get_parameter("bias_anchor_ids").as_integer_array();
        auto bias_ns  = get_parameter("bias_values_ns").as_double_array();
        if (bias_ids.size() != bias_ns.size()) {
            RCLCPP_ERROR(get_logger(),
                "bias_anchor_ids/values size mismatch (%zu vs %zu) — "
                "no bias correction applied",
                bias_ids.size(), bias_ns.size());
        } else {
            for (std::size_t i = 0; i < bias_ids.size(); ++i) {
                bias_s_[static_cast<int>(bias_ids[i])] = bias_ns[i] * 1e-9;
            }
        }

        wolf_cfg_.sigma_tdoa    = get_parameter("wolf_sigma_tdoa").as_double();
        wolf_cfg_.sigma_a_min   = get_parameter("wolf_sigma_a_min").as_double();
        wolf_cfg_.sigma_a_gain  = get_parameter("wolf_sigma_a_gain").as_double();
        wolf_cfg_.sigma_a_max   = get_parameter("wolf_sigma_a_max").as_double();
        wolf_cfg_.wolf_c        = get_parameter("wolf_c").as_double();
        wolf_cfg_.pos_std0      = get_parameter("wolf_pos_std0").as_double();
        wolf_cfg_.vel_std0      = get_parameter("wolf_vel_std0").as_double();
        wolf_cfg_.acc_std0      = get_parameter("wolf_acc_std0").as_double();
        log_every_s_            = get_parameter("wolf_log_every_s").as_double();

        auto pn = get_parameter("pos_nominal").as_double_array();
        auto rd = get_parameter("room_dim").as_double_array();
        if (pn.size() == 3) pos_nominal_ = Eigen::Vector3d(pn[0], pn[1], pn[2]);
        if (rd.size() == 3) room_dim_    = Eigen::Vector3d(rd[0], rd[1], rd[2]);
        dt_nominal_ = get_parameter("dt_nominal_s").as_double();

        last_pos_chan_ = pos_nominal_;

        // ===== ZUPT config (v0.3.0) =====
        ZuptHold::Config zc;
        zc.enabled        = get_parameter("zupt_enabled").as_bool();
        zc.v_enter        = get_parameter("zupt_v_enter").as_double();
        zc.v_exit         = get_parameter("zupt_v_exit").as_double();
        zc.spread_enter   = get_parameter("zupt_spread_enter").as_double();
        zc.window_s       = get_parameter("zupt_window_s").as_double();
        zc.n_enter        = get_parameter("zupt_n_enter").as_int();
        zc.hold_cutoff_hz = get_parameter("zupt_hold_cutoff_hz").as_double();
        zc.ramp_s         = get_parameter("zupt_ramp_s").as_double();
        zc.use_median     =
            (get_parameter("zupt_hold_mode").as_string() == "median");
        zupt_log_every_s_ = get_parameter("zupt_log_every_s").as_double();
        zupt_ = std::make_unique<ZuptHold>(zc);
        zupt_enabled_ = zc.enabled;

        // ===== ROS plumbing =====
        sub_toa_ = create_subscription<ips_msgs::msg::CorrectedToA>(
            "/uwb/corrected_toa", qos_state_reliable(),
            std::bind(&WolfPositionNode::on_toa, this, _1));
        sub_cfg_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/uwb/anchor_config", qos_latched_config(),
            std::bind(&WolfPositionNode::on_anchor_config, this, _1));
        pub_chan_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "/state/position_chan", qos_state_reliable());
        pub_pos_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/state/position", qos_state_reliable());
        pub_vel_  = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/state/wolf_velocity", qos_state_reliable());

        RCLCPP_INFO(get_logger(),
            "wolf_position (C++) v0.3.3 ready — WoLF-EKF-CA 9-state, "
            "IMQ c=%.2f, σ_a=[%.3f..%.2f] gain=%.2f, ZUPT=%s, +wolf_velocity, "
            "+σ_a clamp +divergence guard",
            wolf_cfg_.wolf_c, wolf_cfg_.sigma_a_min, wolf_cfg_.sigma_a_max,
            wolf_cfg_.sigma_a_gain, zupt_enabled_ ? "ON" : "off");
    }

private:
    // ===== anchor_config callback (build Chan geometry + WoLF cfg) =====
    void on_anchor_config(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mu_);

        std::vector<int> ids;
        {
            std::string s = msg->header.frame_id;
            std::size_t pos = 0;
            while (pos < s.size()) {
                std::size_t comma = s.find(',', pos);
                std::string tok = (comma == std::string::npos)
                                ? s.substr(pos)
                                : s.substr(pos, comma - pos);
                if (!tok.empty()) ids.push_back(std::stoi(tok));
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        }
        if (ids.size() != msg->poses.size()) {
            RCLCPP_ERROR(get_logger(),
                "anchor_config: id/pose count mismatch (%zu vs %zu)",
                ids.size(), msg->poses.size());
            return;
        }
        std::map<int, Eigen::Vector3d> anchor_pos;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            anchor_pos[ids[i]] = Eigen::Vector3d(
                msg->poses[i].position.x,
                msg->poses[i].position.y,
                msg->poses[i].position.z);
        }
        std::array<Eigen::Vector3d, 4> ordered;
        for (int i = 0; i < 4; ++i) {
            auto it = anchor_pos.find(rx_ids_[i]);
            if (it == anchor_pos.end()) {
                RCLCPP_ERROR(get_logger(),
                    "anchor_config: missing rx anchor %d", rx_ids_[i]);
                return;
            }
            ordered[i] = it->second;
        }

        // Build Chan geometry for init
        geom_ = build_chan_geometry(ordered);
        if (!geom_.valid) {
            RCLCPP_ERROR(get_logger(),
                "Chan geometry singular — anchors may be coplanar");
            return;
        }

        // Configure WoLF + initialize immediately from pos_nominal_.
        // v0.2.0: tidak menunggu Chan valid. EKF + IMQ akan menarik state
        // ke posisi sebenarnya dalam beberapa blink berikutnya. pos_std0
        // yang besar (default 1.5m) memberi ruang untuk koreksi besar di
        // langkah-langkah awal.
        wolf_cfg_.anchors_ordered = ordered;
        wolf_ = std::make_unique<WolfEkfCa>(wolf_cfg_);
        wolf_->initialize(pos_nominal_);
        geometry_ready_ = true;

        RCLCPP_INFO(get_logger(),
            "anchor_config loaded: %zu anchors, WoLF initialized from "
            "pos_nominal=(%.2f, %.2f, %.2f) with P0_pos_std=%.2f m",
            anchor_pos.size(),
            pos_nominal_(0), pos_nominal_(1), pos_nominal_(2),
            wolf_cfg_.pos_std0);
    }

    // ===== ToA callback (buffer per blink, dispatch when 4 complete) =====
    void on_toa(const ips_msgs::msg::CorrectedToA::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mu_);
        if (!geometry_ready_ || !wolf_) return;

        const int sid = msg->slave_id;
        bool is_rx = false;
        for (int a : rx_ids_) if (a == sid) { is_rx = true; break; }
        if (!is_rx) return;

        const int    seq    = msg->tag_seq;
        const double bias_s = bias_s_.count(sid) ? bias_s_.at(sid) : 0.0;
        const double toa_s  = msg->toa_corrected_s - bias_s;

        const auto now_s = static_cast<double>(now().nanoseconds()) * 1e-9;

        auto it = buf_.find(seq);
        if (it == buf_.end()) {
            while (buf_.size() >= buf_size_) {
                auto oldest = buf_.begin();
                buf_t_.erase(oldest->first);
                buf_.erase(oldest);
            }
            it = buf_.emplace(seq, std::map<int, double>{}).first;
            buf_t_[seq] = now_s;
        }
        it->second[sid] = toa_s;

        if (it->second.size() == 4) {
            solve_and_publish(seq, msg->header.stamp, it->second);
            buf_t_.erase(seq);
            buf_.erase(it);
        }
        for (auto pt = buf_t_.begin(); pt != buf_t_.end(); ) {
            if (now_s - pt->second > timeout_s_) {
                buf_.erase(pt->first);
                pt = buf_t_.erase(pt);
            } else { ++pt; }
        }
        maybe_log(now_s);
    }

    // ===== Process one complete blink =====
    void solve_and_publish(int seq,
                           const builtin_interfaces::msg::Time& stamp,
                           const std::map<int, double>& bucket)
    {
        (void)seq;

        // Bias-corrected ToAs in ordered slots [ref, r1, r2, r3]
        std::array<double, 4> toa_ordered;
        for (int i = 0; i < 4; ++i) toa_ordered[i] = bucket.at(rx_ids_[i]);

        // TDoA in meters relative to reference (anchor at index 0)
        const double          t_ref = toa_ordered[0];
        const Eigen::Vector3d tdoa(
            C_LIGHT_M_S * (toa_ordered[1] - t_ref),
            C_LIGHT_M_S * (toa_ordered[2] - t_ref),
            C_LIGHT_M_S * (toa_ordered[3] - t_ref));

        // dt from reference ToA
        double dt = dt_nominal_;
        if (last_t_ref_.has_value()) {
            const double cand = t_ref - *last_t_ref_;
            if (cand > 0.0 && cand <= 5.0) dt = cand;
        }
        last_t_ref_ = t_ref;

        // Try Chan solve (diagnostic only — published to /state/position_chan
        // for monitoring; WoLF does NOT depend on Chan validity)
        const Eigen::Vector3d lower_b =
            Eigen::Vector3d::Constant(-0.5);
        const Eigen::Vector3d upper_b =
            room_dim_ + Eigen::Vector3d::Constant(0.5);
        auto [pos_chan, chan_valid] = chan_solve(
            toa_ordered, geom_, lower_b, upper_b, last_pos_chan_);
        if (chan_valid) last_pos_chan_ = pos_chan;

        // Publish raw Chan (diagnostic)
        {
            geometry_msgs::msg::PointStamped m;
            m.header.stamp    = stamp;
            m.header.frame_id = frame_id_;
            m.point.x = pos_chan(0);
            m.point.y = pos_chan(1);
            m.point.z = pos_chan(2);
            pub_chan_->publish(m);
        }

        // ===== WoLF predict + update — runs every blink, no init gating =====
        wolf_->predict(dt);
        const double w = wolf_->update(tdoa);

        // ===== Jaring pengaman divergensi (v0.3.3) =====
        // Kalau state meledak (NaN/inf atau posisi absurd jauh di luar ruangan),
        // re-init EKF dari Chan terbaru. Ubah "ledakan merusak sisa sesi" jadi
        // "satu glitch lalu pulih". pos_limit = diagonal ruangan × 3 (longgar).
        const double pos_limit =
            3.0 * (room_dim_.norm() + 1.0);
        if (wolf_->is_diverged(pos_limit)) {
            if (chan_valid) {
                wolf_->initialize(pos_chan);
                RCLCPP_WARN(get_logger(),
                    "WoLF diverged (>%.0fm) — re-init dari Chan [%.2f %.2f %.2f]",
                    pos_limit, pos_chan(0), pos_chan(1), pos_chan(2));
            } else {
                wolf_->initialize(pos_nominal_);
                RCLCPP_WARN(get_logger(),
                    "WoLF diverged, Chan tak valid — re-init dari pos_nominal");
            }
            n_diverged_++;
        }

        // Track weight statistics
        n_updates_++;
        sum_w_ += w;
        if (w < 0.5) n_w_below_half_++;

        const Eigen::Vector3d pos_out = wolf_->position();
        const Eigen::Matrix<double, 9, 9>& P_full = wolf_->covariance();

        // ===== ZUPT post-filter (v0.3.0) =====
        // Saring posisi EKF sebelum publish. State EKF TIDAK disentuh —
        // hanya nilai yang dikeluarkan. Gate pakai ‖v̂‖ dari state EKF
        // (aman terhadap ledakan: saat over-ekstrapolasi ‖v̂‖ tinggi → ZUPT off).
        const double v_norm = wolf_->velocity().norm();
        const Eigen::Vector3d pos_pub =
            zupt_ ? zupt_->process(pos_out, v_norm, dt) : pos_out;

        // Publish position (raw — bias_compensator handles affine downstream)
        {
            geometry_msgs::msg::PoseWithCovarianceStamped m;
            m.header.stamp    = stamp;
            m.header.frame_id = frame_id_;
            m.pose.pose.position.x = pos_pub(0);
            m.pose.pose.position.y = pos_pub(1);
            m.pose.pose.position.z = pos_pub(2);
            m.pose.pose.orientation.w = 1.0;
            // 6x6 covariance: top-left 3x3 = position covariance
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    m.pose.covariance[i * 6 + j] = P_full(i, j);
            pub_pos_->publish(m);
        }

        // Publish kecepatan STATE EKF (v̂) — yang dipakai gate ZUPT.
        // BEDA dari /state/translation_velocity (differentiator = turunan posisi).
        // Berguna untuk analisis offline: bandingkan v_ekf vs v_diff.
        {
            geometry_msgs::msg::Vector3Stamped vmsg;
            vmsg.header.stamp    = stamp;
            vmsg.header.frame_id = frame_id_;
            const Eigen::Vector3d v = wolf_->velocity();
            vmsg.vector.x = v(0);
            vmsg.vector.y = v(1);
            vmsg.vector.z = v(2);
            pub_vel_->publish(vmsg);
        }
    }

    void maybe_log(double now_s) {
        if (log_every_s_ <= 0.0) return;
        if ((now_s - last_log_t_) < log_every_s_) return;
        last_log_t_ = now_s;
        if (!wolf_ || n_updates_ == 0) return;
        const double w_mean   = sum_w_ / static_cast<double>(n_updates_);
        const double w_below  = 100.0 * static_cast<double>(n_w_below_half_)
                                      / static_cast<double>(n_updates_);
        const double v_norm   = wolf_->velocity().norm();
        const Eigen::Vector3d p = wolf_->position();
        RCLCPP_INFO(get_logger(),
            "WoLF: pos=(%.2f,%.2f,%.2f) |v|=%.2fm/s  "
            "w_mean=%.3f  w<0.5: %.1f%%  ZUPT=%s  (n=%ld, diverged=%ld)",
            p(0), p(1), p(2), v_norm, w_mean, w_below,
            (zupt_ && zupt_->holding()) ? "HOLD" : "live", n_updates_, n_diverged_);
    }

    // ===== State =====
    std::mutex mu_;
    std::string                                  frame_id_;
    std::array<int, 4>                           rx_ids_;
    std::map<int, double>                        bias_s_;
    Eigen::Vector3d                              pos_nominal_{1.0, 1.0, 1.75};
    Eigen::Vector3d                              room_dim_{2.0, 2.0, 3.5};
    Eigen::Vector3d                              last_pos_chan_{1.0, 1.0, 1.75};
    double                                       dt_nominal_ = 0.20;
    std::size_t                                  buf_size_   = 64;
    double                                       timeout_s_  = 0.5;
    std::optional<double>                        last_t_ref_;

    bool                                         geometry_ready_ = false;
    ChanGeometry                                 geom_;
    WolfEkfCa::Config                            wolf_cfg_;
    std::unique_ptr<WolfEkfCa>                   wolf_;

    // ZUPT post-filter (v0.3.0)
    std::unique_ptr<ZuptHold>                    zupt_;
    bool                                         zupt_enabled_     = false;
    double                                       zupt_log_every_s_ = 5.0;

    std::map<int, std::map<int, double>>         buf_;
    std::map<int, double>                        buf_t_;

    double                                       log_every_s_   = 5.0;
    double                                       last_log_t_    = 0.0;
    long                                         n_updates_     = 0;
    long                                         n_w_below_half_= 0;
    long                                         n_diverged_    = 0;
    double                                       sum_w_         = 0.0;

    rclcpp::Subscription<ips_msgs::msg::CorrectedToA>::SharedPtr   sub_toa_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_cfg_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_chan_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vel_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pub_pos_;
};

}  // namespace ips

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ips::WolfPositionNode>());
    rclcpp::shutdown();
    return 0;
}
