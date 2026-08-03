// =============================================================================
//  position_solver_node.cpp
//  C++ position solver with full pipeline:
//
//    aggregate 4 ToA → KF predict_only → [Layer 1 Huber clip] → Chan
//      → publish chan → [Layer 2 gate] → KF α-scaled update → publish kf pos
//
//  Layer 1: TDoA-level Huber clipping using KF prediction (Predictive TDoA Gate)
//  Layer 2: Position-level innovation gating (Mahalanobis chi-squared)
//           Applied INSIDE StudentTFilter::step() when kf_type=student_t.
// =============================================================================
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "ips_msgs/msg/corrected_to_a.hpp"

#include "ips_nodes_cpp/chan_solver.hpp"
#include "ips_nodes_cpp/dzs_filter.hpp"
#include "ips_nodes_cpp/position_kf.hpp"
#include "ips_nodes_cpp/predictive_tdoa_gate.hpp"

namespace ips {

using std::placeholders::_1;

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
class PositionSolverNode : public rclcpp::Node {
public:
    PositionSolverNode() : rclcpp::Node("position_solver") {
        // ============= Parameters ====================================
        declare_parameter<std::string>("frame_id", "world");
        declare_parameter<bool>       ("use_kalman_filter", true);
        declare_parameter<int>        ("blink_buffer_size", 64);
        declare_parameter<double>     ("blink_publish_timeout_s", 0.5);
        declare_parameter<std::vector<int64_t>>("rx_anchors_ordered",
                                                {2, 3, 4, 5});
        declare_parameter<std::vector<int64_t>>("bias_anchor_ids",
                                                {2, 3, 4, 5});
        declare_parameter<std::vector<double>>("bias_values_ns",
                                               {0.0, 0.0, 0.0, 0.0});
        declare_parameter<double>("sigma_a",       0.05);
        declare_parameter<double>("sigma_t_tdoa", 2.13e-10);
        declare_parameter<std::vector<double>>("pos_nominal",
                                               {1.75, 1.35, 1.5});
        declare_parameter<std::vector<double>>("room_dim",
                                               {3.4374, 2.74, 3.5});
        declare_parameter<double>     ("dt_nominal_s", 0.20);
        declare_parameter<std::string>("kf_type",      "student_t");
        declare_parameter<double>     ("stf_eta0",     5.0);
        declare_parameter<double>     ("stf_alpha_max", 3.0);

        declare_parameter<bool>  ("dzs_enabled",   false);
        declare_parameter<int>   ("dzs_window",    100);
        declare_parameter<double>("dzs_k_pos",     2.5);
        declare_parameter<double>("dzs_k_neg",     3.5);
        declare_parameter<double>("dzs_mad_min_m", 0.03);
        declare_parameter<int>   ("dzs_warmup",    20);
        declare_parameter<double>("dzs_log_every_s", 5.0);
        declare_parameter<bool>  ("g1_enabled",    true);
        declare_parameter<double>("g1_margin",     1.5);

        // ===== Layer 1 parameters =====
        declare_parameter<bool>  ("layer1_enabled",      true);
        declare_parameter<double>("layer1_sigma_tdoa_m", 0.10);
        declare_parameter<double>("layer1_huber_k",      2.5);
        declare_parameter<int>   ("layer1_warmup",       20);
        declare_parameter<double>("layer1_log_every_s",  5.0);

        // ===== Layer 2 parameters =====
        // gate_threshold = chi-squared(dim=3) inverse-CDF at probability p
        //   p=0.95  → 7.815
        //   p=0.99  → 11.345  (default, ~99% acceptance)
        //   p=0.999 → 16.266
        // Set <= 0 to disable Layer 2 (pure Student-t).
        declare_parameter<bool>  ("layer2_enabled",        true);
        declare_parameter<double>("layer2_gate_threshold", 11.345);
        declare_parameter<double>("layer2_log_every_s",    5.0);

        // ============= Fetch values ==================================
        frame_id_  = get_parameter("frame_id").as_string();
        use_kf_    = get_parameter("use_kalman_filter").as_bool();
        buf_size_  = static_cast<std::size_t>(
                        get_parameter("blink_buffer_size").as_int());
        timeout_s_ = get_parameter("blink_publish_timeout_s").as_double();

        auto rx_v  = get_parameter("rx_anchors_ordered").as_integer_array();
        if (rx_v.size() != 4) {
            throw std::runtime_error("rx_anchors_ordered must have 4 entries");
        }
        for (int i = 0; i < 4; ++i) rx_ids_[i] = static_cast<int>(rx_v[i]);

        auto bias_ids   = get_parameter("bias_anchor_ids").as_integer_array();
        auto bias_ns    = get_parameter("bias_values_ns").as_double_array();
        for (std::size_t i = 0; i < bias_ids.size(); ++i) {
            bias_s_[static_cast<int>(bias_ids[i])] = bias_ns[i] * 1e-9;
        }
        for (int a : rx_ids_) {
            if (bias_s_.find(a) == bias_s_.end()) bias_s_[a] = 0.0;
        }

        sigma_a_     = get_parameter("sigma_a").as_double();
        auto pn      = get_parameter("pos_nominal").as_double_array();
        auto rd      = get_parameter("room_dim").as_double_array();
        pos_nominal_ = Eigen::Vector3d(pn[0], pn[1], pn[2]);
        room_dim_    = Eigen::Vector3d(rd[0], rd[1], rd[2]);
        dt_nominal_  = get_parameter("dt_nominal_s").as_double();

        kf_type_      = get_parameter("kf_type").as_string();
        stf_eta0_     = get_parameter("stf_eta0").as_double();
        stf_alpha_max_= get_parameter("stf_alpha_max").as_double();

        dzs_enabled_   = get_parameter("dzs_enabled").as_bool();
        g1_enabled_    = get_parameter("g1_enabled").as_bool();
        g1_margin_     = get_parameter("g1_margin").as_double();
        dzs_log_every_s_ = get_parameter("dzs_log_every_s").as_double();

        dzs_cfg_.window     = static_cast<std::size_t>(
                                get_parameter("dzs_window").as_int());
        dzs_cfg_.k_pos      = get_parameter("dzs_k_pos").as_double();
        dzs_cfg_.k_neg      = get_parameter("dzs_k_neg").as_double();
        dzs_cfg_.mad_min_m  = get_parameter("dzs_mad_min_m").as_double();
        dzs_cfg_.warmup     = static_cast<std::size_t>(
                                get_parameter("dzs_warmup").as_int());
        dzs_cfg_.g1_enabled = g1_enabled_;
        dzs_cfg_.g1_margin  = g1_margin_;

        layer1_enabled_   = get_parameter("layer1_enabled").as_bool();
        layer1_sigma_tdoa_= get_parameter("layer1_sigma_tdoa_m").as_double();
        layer1_huber_k_   = get_parameter("layer1_huber_k").as_double();
        layer1_warmup_    = static_cast<std::size_t>(
                                get_parameter("layer1_warmup").as_int());
        layer1_log_every_s_ = get_parameter("layer1_log_every_s").as_double();

        layer2_enabled_         = get_parameter("layer2_enabled").as_bool();
        layer2_gate_threshold_  = get_parameter("layer2_gate_threshold").as_double();
        layer2_log_every_s_     = get_parameter("layer2_log_every_s").as_double();

        // ============= ROS plumbing ==================================
        last_pos_chan_ = pos_nominal_;

        sub_toa_ = create_subscription<ips_msgs::msg::CorrectedToA>(
            "/uwb/corrected_toa", qos_state_reliable(),
            std::bind(&PositionSolverNode::on_toa, this, _1));

        sub_cfg_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/uwb/anchor_config", qos_latched_config(),
            std::bind(&PositionSolverNode::on_anchor_config, this, _1));

        pub_chan_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "/state/position_chan", qos_state_reliable());
        pub_kf_   = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/state/position", qos_state_reliable());

        RCLCPP_INFO(get_logger(),
            "position_solver (C++) ready  KF=%s  kf_type=%s  L1=%s  L2=%s  DZS=%s",
            use_kf_ ? "on" : "off", kf_type_.c_str(),
            layer1_enabled_ ? "on" : "off",
            layer2_enabled_ ? "on" : "off",
            dzs_enabled_    ? "on" : "off");
    }

private:
    // =============================================================
    void on_anchor_config(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
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
        anchor_pos_.clear();
        for (std::size_t i = 0; i < ids.size(); ++i) {
            anchor_pos_[ids[i]] = Eigen::Vector3d(
                msg->poses[i].position.x,
                msg->poses[i].position.y,
                msg->poses[i].position.z);
        }
        std::array<Eigen::Vector3d, 4> ordered;
        for (int i = 0; i < 4; ++i) {
            auto it = anchor_pos_.find(rx_ids_[i]);
            if (it == anchor_pos_.end()) {
                RCLCPP_ERROR(get_logger(),
                    "anchor_config: missing position for rx anchor %d",
                    rx_ids_[i]);
                return;
            }
            ordered[i] = it->second;
        }
        geom_ = build_chan_geometry(ordered);
        if (!geom_.valid) {
            RCLCPP_ERROR(get_logger(),
                "Chan geometry singular — check anchor placement");
            return;
        }

        const Eigen::Vector3d& ref = ordered[0];
        for (int i = 1; i < 4; ++i) {
            dzs_cfg_.g1_distance_m[rx_ids_[i]] = (ordered[i] - ref).norm();
        }
        dzs_filter_ = std::make_unique<AsymmetricDzsFilter>(dzs_cfg_);

        // Layer 1
        PredictiveTdoaGate::Config l1cfg;
        l1cfg.rx_ids_ordered = rx_ids_;
        l1cfg.anchors        = ordered;
        l1cfg.sigma_tdoa     = layer1_sigma_tdoa_;
        l1cfg.huber_k        = layer1_huber_k_;
        l1cfg.warmup         = layer1_warmup_;
        layer1_ = std::make_unique<PredictiveTdoaGate>(l1cfg);

        // KF init/reinit — passes Layer 2 gate threshold to Student-t
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.0025;
        if (kf_type_ == "student_t") {
            const double gate = layer2_enabled_
                                ? layer2_gate_threshold_
                                : -1.0;  // negative disables gate
            kf_ = std::make_unique<StudentTFilter>(
                R, pos_nominal_, sigma_a_, stf_eta0_, stf_alpha_max_, gate);
        } else {
            kf_ = std::make_unique<PositionKF>(R, pos_nominal_, sigma_a_);
        }
        RCLCPP_INFO(get_logger(),
            "anchor_config loaded: %zu anchors, pipeline ready",
            anchor_pos_.size());
    }

    // =============================================================
    void on_toa(const ips_msgs::msg::CorrectedToA::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mu_);
        if (!geom_.valid || !kf_) return;

        const int sid = msg->slave_id;
        bool is_rx = false;
        for (int a : rx_ids_) if (a == sid) { is_rx = true; break; }
        if (!is_rx) return;

        const int    seq    = msg->tag_seq;
        const double bias   = bias_s_.count(sid) ? bias_s_.at(sid) : 0.0;
        const double toa_s  = msg->toa_corrected_s - bias;

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
            } else {
                ++pt;
            }
        }
        maybe_log_dzs(now_s);
        maybe_log_layer1(now_s);
        maybe_log_layer2(now_s);
    }

    // =============================================================
    void solve_and_publish(int seq,
                           const builtin_interfaces::msg::Time& stamp,
                           const std::map<int, double>& bucket) {
        (void)seq;

        std::array<double, 4> toa_ordered;
        for (int i = 0; i < 4; ++i) toa_ordered[i] = bucket.at(rx_ids_[i]);

        const double t_ref = toa_ordered[0];
        double dt = dt_nominal_;
        if (last_t_ref_.has_value()) {
            const double cand = t_ref - *last_t_ref_;
            if (cand > 0.0 && cand <= 5.0) dt = cand;
        }
        last_t_ref_ = t_ref;

        // ---- Layer 1: TDoA-level Huber clipping ----
        if (layer1_enabled_ && layer1_ && use_kf_ && kf_) {
            Eigen::Vector3d x_pred;
            Eigen::Matrix3d P_pos;
            kf_->predict_only(dt, x_pred, P_pos);
            layer1_->filter(toa_ordered, x_pred, P_pos);
        }

        // ---- DZS / G1 (optional) ----
        if (dzs_enabled_ && dzs_filter_) {
            std::map<int, double> tdoa_m;
            const double ref = toa_ordered[0];
            for (int i = 1; i < 4; ++i)
                tdoa_m[rx_ids_[i]] = C_MS_S * (toa_ordered[i] - ref);
            std::string reason;
            if (!dzs_filter_->check_and_update(tdoa_m, &reason)) {
                RCLCPP_DEBUG(get_logger(),
                    "DZS/G1 reject seq=%d: %s", seq, reason.c_str());
                return;
            }
        }

        // ---- Chan ----
        const Eigen::Vector3d lower_b = Eigen::Vector3d::Constant(-0.5);
        const Eigen::Vector3d upper_b = room_dim_ + Eigen::Vector3d::Constant(0.5);
        auto [pos_chan, valid] = chan_solve(
            toa_ordered, geom_, lower_b, upper_b, last_pos_chan_);

        {
            geometry_msgs::msg::PointStamped m;
            m.header.stamp    = stamp;
            m.header.frame_id = frame_id_;
            m.point.x = pos_chan(0);
            m.point.y = pos_chan(1);
            m.point.z = pos_chan(2);
            pub_chan_->publish(m);
        }

        const Eigen::Vector3d z_in = valid ? pos_chan : last_pos_chan_;
        if (valid) last_pos_chan_ = pos_chan;

        // ---- KF update (Layer 2 gate happens INSIDE StudentTFilter::step) ----
        Eigen::Vector3d pos_out = z_in;
        if (use_kf_ && kf_) {
            pos_out = kf_->step(z_in, dt);
        }

        {
            geometry_msgs::msg::PoseWithCovarianceStamped m;
            m.header.stamp    = stamp;
            m.header.frame_id = frame_id_;
            m.pose.pose.position.x = pos_out(0);
            m.pose.pose.position.y = pos_out(1);
            m.pose.pose.position.z = pos_out(2);
            m.pose.pose.orientation.w = 1.0;
            if (use_kf_ && kf_) {
                const auto& P = kf_->cov();
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        m.pose.covariance[i * 6 + j] = P(i, j);
            }
            pub_kf_->publish(m);
        }
    }

    void maybe_log_dzs(double now_s) {
        if (!dzs_enabled_ || !dzs_filter_) return;
        if (dzs_log_every_s_ <= 0.0) return;
        if ((now_s - last_dzs_log_t_) < dzs_log_every_s_) return;
        last_dzs_log_t_ = now_s;
        const auto& s = dzs_filter_->stats();
        if (s.n_total == 0) return;
        RCLCPP_INFO(get_logger(),
            "DZS stats: n=%ld inliers=%ld outliers=%ld g1_rej=%ld (%.1f%%)",
            s.n_total, s.n_inliers, s.n_outliers, s.n_g1_reject,
            100.0 * s.reject_rate());
    }

    void maybe_log_layer1(double now_s) {
        if (!layer1_enabled_ || !layer1_) return;
        if (layer1_log_every_s_ <= 0.0) return;
        if ((now_s - last_layer1_log_t_) < layer1_log_every_s_) return;
        last_layer1_log_t_ = now_s;
        RCLCPP_INFO(get_logger(), "%s", layer1_->stats_summary().c_str());
    }

    void maybe_log_layer2(double now_s) {
        if (!layer2_enabled_ || !kf_) return;
        if (kf_type_ != "student_t") return;
        if (layer2_log_every_s_ <= 0.0) return;
        if ((now_s - last_layer2_log_t_) < layer2_log_every_s_) return;
        last_layer2_log_t_ = now_s;
        const auto* stf = dynamic_cast<const StudentTFilter*>(kf_.get());
        if (!stf) return;
        const long tot = stf->n_total();
        if (tot == 0) return;
        const long gtd = stf->n_gated();
        const double pct = 100.0 * static_cast<double>(gtd) / tot;
        RCLCPP_INFO(get_logger(),
            "[Layer 2] innovation gate: rejected %ld/%ld (%.2f%%)  "
            "last_NIS=%.2f  threshold=%.2f",
            gtd, tot, pct, stf->last_nis(), stf->gate_threshold());
    }

    // ---- State ---------------------------------------------------
    std::string                                  frame_id_;
    bool                                         use_kf_ = true;
    std::array<int, 4>                           rx_ids_{};
    std::map<int, double>                        bias_s_;
    std::map<int, Eigen::Vector3d>               anchor_pos_;
    Eigen::Vector3d                              pos_nominal_, room_dim_;
    Eigen::Vector3d                              last_pos_chan_;
    std::optional<double>                        last_t_ref_;
    double                                       sigma_a_, dt_nominal_;
    std::string                                  kf_type_;
    double                                       stf_eta0_, stf_alpha_max_;

    bool                                         dzs_enabled_, g1_enabled_;
    double                                       g1_margin_, dzs_log_every_s_;
    double                                       last_dzs_log_t_ = 0.0;
    AsymmetricDzsFilter::Config                  dzs_cfg_;
    std::unique_ptr<AsymmetricDzsFilter>         dzs_filter_;

    bool                                         layer1_enabled_ = true;
    double                                       layer1_sigma_tdoa_ = 0.10;
    double                                       layer1_huber_k_    = 2.5;
    std::size_t                                  layer1_warmup_     = 20;
    double                                       layer1_log_every_s_= 5.0;
    double                                       last_layer1_log_t_ = 0.0;
    std::unique_ptr<PredictiveTdoaGate>          layer1_;

    bool                                         layer2_enabled_         = true;
    double                                       layer2_gate_threshold_  = 11.345;
    double                                       layer2_log_every_s_     = 5.0;
    double                                       last_layer2_log_t_      = 0.0;

    ChanGeometry                                 geom_;
    std::unique_ptr<PositionKF>                  kf_;

    std::size_t                                  buf_size_ = 64;
    double                                       timeout_s_ = 0.5;
    std::map<int, std::map<int, double>>         buf_;
    std::map<int, double>                        buf_t_;

    std::mutex                                   mu_;

    rclcpp::Subscription<ips_msgs::msg::CorrectedToA>::SharedPtr        sub_toa_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr      sub_cfg_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr      pub_chan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_kf_;
};

} // namespace ips

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ips::PositionSolverNode>());
    rclcpp::shutdown();
    return 0;
}
