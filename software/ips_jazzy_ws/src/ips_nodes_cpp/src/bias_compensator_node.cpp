// =============================================================================
//  bias_compensator_node.cpp  (v2 — affine + orientation)
//
//  Menggabungkan dua patch sebelumnya:
//    - pose_orient_patch : lampirkan orientasi IMU ke Pose output (RViz)
//    - affine_bias_patch : model bias affine penuh  p_true = M * p_meas + b
//
//  Backward-compatible: membaca format YAML lama (bias:{x,y,z}) maupun
//  format affine baru (bias_model: affine, bias_matrix, bias_offset).
//
//  State log startup menampilkan mode=AFFINE atau mode=OFFSET.
//  Saat mode AFFINE, kalibrasi service (satu titik GT) memperbarui
//  offset b saja (M tetap dari YAML). Untuk refit M penuh: fit_bias.py.
// =============================================================================
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "ips_msgs/srv/calibrate.hpp"

namespace fs = std::filesystem;
namespace ips {

using PoseStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using std::placeholders::_1;
using std::placeholders::_2;

static rclcpp::QoS qos_state_reliable() {
    rclcpp::QoS q(20); q.reliable().keep_last(20); return q;
}

// ─── Statistik per-sumbu ────────────────────────────────────────────────────
static Eigen::Vector3d per_axis_median(const std::vector<Eigen::Vector3d>& s) {
    Eigen::Vector3d med = Eigen::Vector3d::Zero();
    if (s.empty()) return med;
    std::vector<double> vals(s.size());
    for (int a = 0; a < 3; ++a) {
        for (size_t i = 0; i < s.size(); ++i) vals[i] = s[i](a);
        const size_t mid = vals.size() / 2;
        std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
        med(a) = vals[mid];
    }
    return med;
}
static Eigen::Vector3d per_axis_mean(const std::vector<Eigen::Vector3d>& s) {
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
    if (s.empty()) return m;
    for (const auto& v : s) m += v;
    return m / static_cast<double>(s.size());
}
static Eigen::Vector3d per_axis_mad(const std::vector<Eigen::Vector3d>& s,
                                    const Eigen::Vector3d& center) {
    Eigen::Vector3d mad = Eigen::Vector3d::Zero();
    if (s.empty()) return mad;
    std::vector<double> vals(s.size());
    for (int a = 0; a < 3; ++a) {
        for (size_t i = 0; i < s.size(); ++i)
            vals[i] = std::fabs(s[i](a) - center(a));
        const size_t mid = vals.size() / 2;
        std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
        mad(a) = vals[mid];
    }
    return mad;
}
static Eigen::Vector3d per_axis_std(const std::vector<Eigen::Vector3d>& s,
                                    const Eigen::Vector3d& mean) {
    Eigen::Vector3d var = Eigen::Vector3d::Zero();
    if (s.size() < 2) return var;
    for (const auto& v : s) { const Eigen::Vector3d d = v - mean; var += d.cwiseProduct(d); }
    var /= static_cast<double>(s.size() - 1);
    return Eigen::Vector3d(std::sqrt(var(0)), std::sqrt(var(1)), std::sqrt(var(2)));
}
static std::string expand_home(const std::string& path) {
    if (!path.empty() && path[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) return std::string(home) + path.substr(1);
    }
    return path;
}
static std::string iso_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf; localtime_r(&t, &tm_buf);
    char buf[32]; std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_buf);
    return std::string(buf);
}

// =============================================================================
class BiasCompensatorNode : public rclcpp::Node {
public:
    enum State     { IDLE = 0, CALIBRATING = 1, OPERATIONAL = 2 };
    enum BiasModel { OFFSET = 0, AFFINE = 1 };

    BiasCompensatorNode() : rclcpp::Node("bias_compensator") {
        declare_parameter<std::string>("bias_yaml_path",        "");
        declare_parameter<bool>       ("auto_load",             true);
        declare_parameter<bool>       ("auto_save",             true);
        declare_parameter<int>        ("n_samples_default",     300);
        declare_parameter<int>        ("skip_warmup_default",   30);
        declare_parameter<bool>       ("robust",                true);
        declare_parameter<double>     ("log_status_every_s",    5.0);
        declare_parameter<double>     ("calibration_timeout_s", 60.0);

        yaml_path_       = get_parameter("bias_yaml_path").as_string();
        auto_load_       = get_parameter("auto_load").as_bool();
        auto_save_       = get_parameter("auto_save").as_bool();
        n_default_       = get_parameter("n_samples_default").as_int();
        skip_default_    = get_parameter("skip_warmup_default").as_int();
        robust_          = get_parameter("robust").as_bool();
        log_every_       = get_parameter("log_status_every_s").as_double();
        calib_timeout_s_ = get_parameter("calibration_timeout_s").as_double();

        // Default: passthrough (M=I, b=0)
        M_        = Eigen::Matrix3d::Identity();
        b_        = Eigen::Vector3d::Zero();
        b_std_    = Eigen::Vector3d::Zero();
        calib_gt_ = Eigen::Vector3d::Zero();

        if (auto_load_ && !yaml_path_.empty()) try_load_bias();

        sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cb_group_;

        sub_pos_ = create_subscription<PoseStamped>(
            "/state/position", qos_state_reliable(),
            std::bind(&BiasCompensatorNode::on_position, this, _1), sub_opts);

        pub_pos_ = create_publisher<PoseStamped>(
            "/state/position_compensated", qos_state_reliable());

        // Orientasi IMU: dilampirkan ke Pose output untuk visualisasi RViz.
        // Tidak sinkron presisi per-blink — cukup untuk visual, bukan metrik.
        sub_orient_ = create_subscription<
            geometry_msgs::msg::QuaternionStamped>(
            "/state/orientation", qos_state_reliable(),
            std::bind(&BiasCompensatorNode::on_orientation, this, _1));

        srv_calib_ = create_service<ips_msgs::srv::Calibrate>(
            "~/calibrate",
            std::bind(&BiasCompensatorNode::on_calibrate, this, _1, _2),
            rclcpp::ServicesQoS(), srv_cb_group_);

        if (log_every_ > 0.0) {
            log_timer_ = create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(log_every_ * 1000)),
                std::bind(&BiasCompensatorNode::log_status, this));
        }

        RCLCPP_INFO(get_logger(),
            "bias_compensator (C++) ready  state=%s  mode=%s  "
            "b=(%+.2f,%+.2f,%+.2f) cm",
            state_name().c_str(), model_name().c_str(),
            b_(0)*100, b_(1)*100, b_(2)*100);
    }

private:
    std::string state_name() const {
        switch (state_) {
            case IDLE:        return "IDLE";
            case CALIBRATING: return "CALIBRATING";
            case OPERATIONAL: return "OPERATIONAL";
        } return "?";
    }
    std::string model_name() const {
        return (bias_model_ == AFFINE) ? "AFFINE" : "OFFSET";
    }

    // Terapkan model: AFFINE = M*p+b, OFFSET = p-b
    Eigen::Vector3d apply_bias(const Eigen::Vector3d& p) const {
        if (bias_model_ == AFFINE) return M_ * p + b_;
        return p - b_;
    }

    // ─── Callback orientasi IMU ───────────────────────────────────────────
    void on_orientation(
            const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(orient_mu_);
        last_orient_ = msg->quaternion;
        have_orient_ = true;
    }

    // ─── Callback posisi UWB ──────────────────────────────────────────────
    void on_position(const PoseStamped::SharedPtr msg) {
        const Eigen::Vector3d pos(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
        // Kumpulkan sampel kalibrasi
        {
            std::lock_guard<std::mutex> lk(calib_mu_);
            if (calib_running_) {
                ++calib_warmup_count_;
                if (calib_warmup_count_ > calib_skip_) {
                    calib_samples_.push_back(pos);
                    if (calib_samples_.size() >=
                            static_cast<size_t>(calib_n_target_ - calib_skip_))
                        finalize_calibration_locked();
                }
            }
        }
        // Terapkan bias
        Eigen::Vector3d corrected;
        { std::lock_guard<std::mutex> lk(state_mu_); corrected = apply_bias(pos); }

        PoseStamped out;
        out.header = msg->header;
        out.pose.pose.position.x = corrected(0);
        out.pose.pose.position.y = corrected(1);
        out.pose.pose.position.z = corrected(2);
        {
            std::lock_guard<std::mutex> lk(orient_mu_);
            out.pose.pose.orientation = have_orient_
                ? last_orient_ : msg->pose.pose.orientation;
        }
        out.pose.covariance = msg->pose.covariance;
        pub_pos_->publish(out);
    }

    // ─── Service kalibrasi ────────────────────────────────────────────────
    void on_calibrate(
            const std::shared_ptr<ips_msgs::srv::Calibrate::Request> req,
            std::shared_ptr<ips_msgs::srv::Calibrate::Response> resp) {
        {
            std::lock_guard<std::mutex> lk(calib_mu_);
            if (calib_running_) {
                resp->success = false; resp->message = "already calibrating";
                return;
            }
            calib_samples_.clear();
            calib_gt_           = { req->gt_x, req->gt_y, req->gt_z };
            calib_n_target_     = (req->n_samples > 0) ? req->n_samples : n_default_;
            calib_skip_         = (req->skip_warmup > 0) ? req->skip_warmup : skip_default_;
            calib_warmup_count_ = 0;
            calib_done_         = false;
            calib_running_      = true;
        }
        RCLCPP_INFO(get_logger(), "[CALIBRATING] gt=(%.3f,%.3f,%.3f) n=%u skip=%u",
            req->gt_x, req->gt_y, req->gt_z, calib_n_target_, calib_skip_);

        {
            std::unique_lock<std::mutex> lk(calib_mu_);
            const bool ok = calib_done_cv_.wait_for(
                lk, std::chrono::duration<double>(calib_timeout_s_),
                [this]{ return calib_done_; });
            if (!ok) {
                calib_running_ = false;
                resp->success  = false; resp->message = "timeout"; return;
            }
        }
        std::lock_guard<std::mutex> lk(state_mu_);
        resp->success = true; resp->message = "ok";
        resp->bias_x  = b_(0); resp->bias_y = b_(1); resp->bias_z = b_(2);
        resp->std_x   = b_std_(0); resp->std_y = b_std_(1); resp->std_z = b_std_(2);
    }

    // ─── Finalisasi kalibrasi ─────────────────────────────────────────────
    void finalize_calibration_locked() {
        Eigen::Vector3d new_b, new_std;
        std::vector<Eigen::Vector3d> residuals;
        residuals.reserve(calib_samples_.size());

        if (bias_model_ == AFFINE) {
            // Perbarui b saja: b_new = mean/median(GT - M*p_meas). M tetap.
            for (const auto& p : calib_samples_)
                residuals.push_back(calib_gt_ - M_ * p);
        } else {
            // OFFSET: b = mean/median(p_meas - GT)
            for (const auto& p : calib_samples_)
                residuals.push_back(p - calib_gt_);
        }
        if (robust_) {
            new_b   = per_axis_median(residuals);
            new_std = 1.4826 * per_axis_mad(residuals, new_b);
        } else {
            new_b   = per_axis_mean(residuals);
            new_std = per_axis_std(residuals, new_b);
        }
        {
            std::lock_guard<std::mutex> lk(state_mu_);
            b_ = new_b; b_std_ = new_std; state_ = OPERATIONAL;
        }
        calib_running_ = false; calib_done_ = true;
        RCLCPP_INFO(get_logger(),
            "Calibration done: mode=%s b=(%+.2f,%+.2f,%+.2f)cm "
            "std=(%.1f,%.1f,%.1f)mm n=%zu",
            model_name().c_str(),
            new_b(0)*100, new_b(1)*100, new_b(2)*100,
            new_std(0)*1000, new_std(1)*1000, new_std(2)*1000,
            calib_samples_.size());
        if (auto_save_ && !yaml_path_.empty()) try_save_bias();
        calib_done_cv_.notify_all();
    }

    // ─── YAML load ────────────────────────────────────────────────────────
    void try_load_bias() {
        try {
            const fs::path path = expand_home(yaml_path_);
            if (!fs::exists(path)) {
                RCLCPP_INFO(get_logger(),
                    "bias yaml tidak ada di %s — mulai IDLE", path.string().c_str());
                return;
            }
            YAML::Node data = YAML::LoadFile(path.string());
            const bool is_affine = data["bias_model"] &&
                data["bias_model"].as<std::string>() == "affine";

            if (is_affine) {
                if (!data["bias_matrix"] || !data["bias_offset"]) {
                    RCLCPP_WARN(get_logger(),
                        "bias yaml affine tidak lengkap (perlu bias_matrix + bias_offset)");
                    return;
                }
                const auto& rows = data["bias_matrix"];
                if (rows.size() != 3) {
                    RCLCPP_WARN(get_logger(), "bias_matrix harus 3 baris"); return;
                }
                Eigen::Matrix3d M_load = Eigen::Matrix3d::Zero();
                for (int r = 0; r < 3; ++r) {
                    if (rows[r].size() != 3) {
                        RCLCPP_WARN(get_logger(),
                            "bias_matrix baris %d harus 3 elemen", r); return;
                    }
                    for (int c = 0; c < 3; ++c)
                        M_load(r, c) = rows[r][c].as<double>();
                }
                const auto& off = data["bias_offset"];
                const Eigen::Vector3d b_load(
                    off["x"].as<double>(), off["y"].as<double>(), off["z"].as<double>());
                {
                    std::lock_guard<std::mutex> lk(state_mu_);
                    M_ = M_load; b_ = b_load;
                    bias_model_ = AFFINE; state_ = OPERATIONAL;
                }
                RCLCPP_INFO(get_logger(),
                    "Loaded AFFINE dari %s: M_diag=(%.4f,%.4f,%.4f) b=(%+.3f,%+.3f,%+.3f)",
                    path.string().c_str(),
                    M_load(0,0), M_load(1,1), M_load(2,2),
                    b_load(0), b_load(1), b_load(2));

            } else if (data["bias"]) {
                const YAML::Node bn = data["bias"];
                const Eigen::Vector3d b_load(
                    bn["x"] ? bn["x"].as<double>() : 0.0,
                    bn["y"] ? bn["y"].as<double>() : 0.0,
                    bn["z"] ? bn["z"].as<double>() : 0.0);
                {
                    std::lock_guard<std::mutex> lk(state_mu_);
                    M_ = Eigen::Matrix3d::Identity(); b_ = b_load;
                    b_std_(0) = bn["std_x"] ? bn["std_x"].as<double>() : 0.0;
                    b_std_(1) = bn["std_y"] ? bn["std_y"].as<double>() : 0.0;
                    b_std_(2) = bn["std_z"] ? bn["std_z"].as<double>() : 0.0;
                    bias_model_ = OFFSET;
                    if (b_load.norm() > 0.0) state_ = OPERATIONAL;
                }
                RCLCPP_INFO(get_logger(),
                    "Loaded OFFSET dari %s: b=(%+.2f,%+.2f,%+.2f) cm",
                    path.string().c_str(), b_load(0)*100, b_load(1)*100, b_load(2)*100);
            } else {
                RCLCPP_WARN(get_logger(),
                    "bias yaml tidak dikenali — tidak ada 'bias' atau "
                    "'bias_model: affine'. File: %s", path.string().c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "load_bias gagal: %s", e.what());
        }
    }

    // ─── YAML save ────────────────────────────────────────────────────────
    void try_save_bias() {
        try {
            const fs::path path = expand_home(yaml_path_);
            fs::create_directories(path.parent_path());
            YAML::Emitter out;
            out << YAML::BeginMap;
            if (bias_model_ == AFFINE) {
                out << YAML::Key << "bias_model" << YAML::Value << "affine";
                out << YAML::Key << "bias_matrix" << YAML::Value << YAML::BeginSeq;
                for (int r = 0; r < 3; ++r) {
                    out << YAML::Flow << YAML::BeginSeq;
                    for (int c = 0; c < 3; ++c) out << M_(r, c);
                    out << YAML::EndSeq;
                }
                out << YAML::EndSeq;
                out << YAML::Key << "bias_offset" << YAML::Value << YAML::BeginMap;
                out << YAML::Key << "x" << YAML::Value << b_(0);
                out << YAML::Key << "y" << YAML::Value << b_(1);
                out << YAML::Key << "z" << YAML::Value << b_(2);
                out << YAML::EndMap;
            } else {
                out << YAML::Key << "bias" << YAML::Value << YAML::BeginMap;
                out << YAML::Key << "x"     << YAML::Value << b_(0);
                out << YAML::Key << "y"     << YAML::Value << b_(1);
                out << YAML::Key << "z"     << YAML::Value << b_(2);
                out << YAML::Key << "std_x" << YAML::Value << b_std_(0);
                out << YAML::Key << "std_y" << YAML::Value << b_std_(1);
                out << YAML::Key << "std_z" << YAML::Value << b_std_(2);
                out << YAML::EndMap;
                out << YAML::Key << "calibration_gt" << YAML::Value << YAML::BeginMap;
                out << YAML::Key << "x" << YAML::Value << calib_gt_(0);
                out << YAML::Key << "y" << YAML::Value << calib_gt_(1);
                out << YAML::Key << "z" << YAML::Value << calib_gt_(2);
                out << YAML::EndMap;
            }
            out << YAML::Key << "timestamp" << YAML::Value << iso_timestamp();
            out << YAML::EndMap;
            std::ofstream f(path); f << out.c_str() << "\n";
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "save_bias gagal: %s", e.what());
        }
    }

    // ─── Periodic log ─────────────────────────────────────────────────────
    void log_status() {
        State st; Eigen::Vector3d b;
        { std::lock_guard<std::mutex> lk(state_mu_); st = state_; b = b_; }
        if (st == CALIBRATING) {
            std::lock_guard<std::mutex> lk(calib_mu_);
            RCLCPP_INFO(get_logger(), "[CALIBRATING] %zu/%zu samples",
                calib_samples_.size(),
                static_cast<size_t>(calib_n_target_ - calib_skip_));
        } else if (st == OPERATIONAL) {
            RCLCPP_INFO(get_logger(),
                "[OPERATIONAL] mode=%s  b=(%+.2f,%+.2f,%+.2f) cm",
                model_name().c_str(), b(0)*100, b(1)*100, b(2)*100);
        }
    }

    // ─── Members ──────────────────────────────────────────────────────────
    std::string yaml_path_;
    bool   auto_load_ = true, auto_save_ = true, robust_ = true;
    int    n_default_ = 300, skip_default_ = 30;
    double log_every_ = 5.0, calib_timeout_s_ = 60.0;

    mutable std::mutex state_mu_;
    Eigen::Matrix3d    M_;            // matriks affine 3×3
    Eigen::Vector3d    b_;            // offset affine (atau offset-only)
    Eigen::Vector3d    b_std_;
    BiasModel          bias_model_ = OFFSET;
    State              state_      = IDLE;

    mutable std::mutex            calib_mu_;
    std::condition_variable       calib_done_cv_;
    bool                          calib_running_      = false;
    bool                          calib_done_         = false;
    Eigen::Vector3d               calib_gt_;
    std::vector<Eigen::Vector3d>  calib_samples_;
    uint32_t calib_n_target_ = 0, calib_skip_ = 0, calib_warmup_count_ = 0;

    geometry_msgs::msg::Quaternion last_orient_;
    bool                           have_orient_ = false;
    std::mutex                     orient_mu_;

    rclcpp::CallbackGroup::SharedPtr sub_cb_group_, srv_cb_group_;
    rclcpp::Subscription<PoseStamped>::SharedPtr sub_pos_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_orient_;
    rclcpp::Publisher<PoseStamped>::SharedPtr  pub_pos_;
    rclcpp::Service<ips_msgs::srv::Calibrate>::SharedPtr srv_calib_;
    rclcpp::TimerBase::SharedPtr               log_timer_;
};

} // namespace ips

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ips::BiasCompensatorNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    try { exec.spin(); }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("bias_compensator"),
                     "spin failed: %s", e.what());
    }
    rclcpp::shutdown(); return 0;
}
