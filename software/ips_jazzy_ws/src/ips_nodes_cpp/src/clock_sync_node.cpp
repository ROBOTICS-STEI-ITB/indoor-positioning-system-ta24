// =============================================================================
//  clock_sync_node.cpp
//  Drop-in C++ replacement for Python clock_sync_node.py.
//
//  Wraps SyncEngine in an rclcpp::Node. Subscribes to /uwb/anchor_reports +
//  /uwb/session_events, publishes /uwb/corrected_toa + /uwb/sync_status.
//
//  Performance vs Python version:
//    - No print/cout in hot path (was 264+ calls/sec @ 25Hz blink)
//    - No internal engine lock (single-threaded executor serializes)
//    - process_pending_tags only for the relevant slave (4× less work)
//    - Fixed-size Eigen matrices (no heap alloc per blink)
//    - Glitch protection + rollback (fixes 2 mm bias bug)
// =============================================================================
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ips_msgs/msg/corrected_to_a.hpp"
#include "ips_msgs/msg/session_event.hpp"
#include "ips_msgs/msg/sync_status.hpp"
#include "ips_msgs/msg/uwb_anchor_report.hpp"

#include "ips_nodes_cpp/dw1000_constants.hpp"
#include "ips_nodes_cpp/sync_engine.hpp"

namespace ips {

using std::placeholders::_1;

// QoS profiles (must match Python ips_nodes/common.py)
static rclcpp::QoS qos_sensor_best_effort_deep() {
    rclcpp::QoS q(500);                          // bumped from 200 for 25Hz blink
    q.best_effort().keep_last(500);
    return q;
}
static rclcpp::QoS qos_state_reliable() {
    rclcpp::QoS q(20);
    q.reliable().keep_last(20);
    return q;
}

class ClockSyncNode : public rclcpp::Node {
public:
    ClockSyncNode() : rclcpp::Node("clock_sync") {
        // ---- Parameters ----------------------------------------------
        declare_parameter<double>("status_period_s", 1.0);
        declare_parameter<bool>  ("kf_enabled",      true);

        const double status_period =
            get_parameter("status_period_s").as_double();
        const bool   kf_enabled    =
            get_parameter("kf_enabled").as_bool();

        // ---- Engine setup --------------------------------------------
        SyncEngine::Config cfg;
        cfg.kf_enabled = kf_enabled;
        engine_ = std::make_unique<SyncEngine>(cfg);

        engine_->set_toa_callback(
            [this](const CorrectedToAResult& r) { on_corrected_toa(r); });

        engine_->set_log_callback(
            [this](int sev, const std::string& msg) {
                switch (sev) {
                    case 0: RCLCPP_DEBUG(get_logger(), "%s", msg.c_str()); break;
                    case 1: RCLCPP_INFO (get_logger(), "%s", msg.c_str()); break;
                    case 2: RCLCPP_WARN (get_logger(), "%s", msg.c_str()); break;
                    default: RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
                }
            });

        // ---- Publishers ----------------------------------------------
        pub_toa_ = create_publisher<ips_msgs::msg::CorrectedToA>(
            "/uwb/corrected_toa", qos_state_reliable());
        pub_status_ = create_publisher<ips_msgs::msg::SyncStatus>(
            "/uwb/sync_status", qos_state_reliable());

        // ---- Subscriptions -------------------------------------------
        sub_reports_ = create_subscription<ips_msgs::msg::UwbAnchorReport>(
            "/uwb/anchor_reports", qos_sensor_best_effort_deep(),
            std::bind(&ClockSyncNode::on_report, this, _1));

        sub_events_ = create_subscription<ips_msgs::msg::SessionEvent>(
            "/uwb/session_events", qos_state_reliable(),
            std::bind(&ClockSyncNode::on_event, this, _1));

        // ---- Status timer --------------------------------------------
        status_timer_ = create_wall_timer(
            std::chrono::milliseconds(
                static_cast<int>(status_period * 1000)),
            std::bind(&ClockSyncNode::publish_status, this));

        RCLCPP_INFO(get_logger(),
            "clock_sync (C++) ready (KF=%s)",
            kf_enabled ? "on" : "off");
    }

private:
    // ---- Report dispatch ---------------------------------------------
    void on_report(const ips_msgs::msg::UwbAnchorReport::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(engine_mu_);
        try {
            switch (msg->report_type) {
                case ips_msgs::msg::UwbAnchorReport::TYPE_MASTER_CLOCK:
                    engine_->add_master_clock(
                        msg->reporter_id, msg->session_id,
                        msg->seq, msg->tx_hex);
                    break;
                case ips_msgs::msg::UwbAnchorReport::TYPE_SLAVE_MASTER:
                    engine_->add_slave_master_rx(
                        msg->reporter_id, msg->source_id,
                        msg->seq, msg->rx_hex);
                    break;
                case ips_msgs::msg::UwbAnchorReport::TYPE_SLAVE_TAG:
                    engine_->add_slave_tag_rx(
                        msg->reporter_id, msg->source_id,
                        msg->seq, msg->rx_hex);
                    break;
                default:
                    break;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "engine ingest error: %s", e.what());
        }
    }

    void on_event(const ips_msgs::msg::SessionEvent::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(engine_mu_);
        if (msg->kind == ips_msgs::msg::SessionEvent::KIND_RESET ||
            msg->kind == ips_msgs::msg::SessionEvent::KIND_AUTO_DETECTED_RESTART) {
            engine_->handle_ma_reset(msg->device_id, msg->session_id);
        } else if (msg->kind == ips_msgs::msg::SessionEvent::KIND_HELLO) {
            engine_->handle_ma_hello(msg->device_id, msg->session_id);
        }
    }

    // ---- Engine callback: publish corrected ToA ----------------------
    // Runs INSIDE on_report under engine_mu_ — already serialized.
    void on_corrected_toa(const CorrectedToAResult& r) {
        ips_msgs::msg::CorrectedToA m;
        m.header.stamp    = now();
        m.header.frame_id = "uwb";
        m.tag_id          = r.tag_id;
        m.slave_id        = r.slave_id;
        m.tag_seq         = r.tag_seq;
        m.sync_seq        = r.sync_seq;
        m.toa_corrected_s = static_cast<double>(r.toa_final_dtu) * DTU_SECONDS;
        m.toa_li_s        = static_cast<double>(r.toa_li_dtu)    * DTU_SECONDS;
        m.toa_kf_s        = static_cast<double>(r.toa_kf_dtu)    * DTU_SECONDS;
        m.prop_delay_s    = static_cast<double>(r.prop_delay_dtu)* DTU_SECONDS;
        m.delta_k         = r.delta_k;
        m.theta_k_s       = static_cast<double>(r.theta_k_dtu)   * DTU_SECONDS;
        m.kf_used         = r.kf_used;
        pub_toa_->publish(m);
    }

    // ---- Status snapshot ---------------------------------------------
    void publish_status() {
        ips_msgs::msg::SyncStatus msg;
        {
            std::lock_guard<std::mutex> lock(engine_mu_);
            const auto status      = engine_->get_sync_status();
            msg.reset_count             = engine_->reset_count();
            msg.dropped_packets_on_reset = engine_->dropped_packets_on_reset();
            for (const auto& [sid, st] : status) {
                msg.slave_ids.push_back(static_cast<uint8_t>(sid));
                msg.sync_count.push_back(st.sync_count);
                msg.li_ready.push_back(st.li_ready);
                msg.kf_converged.push_back(st.kf_converged);
                msg.kf_updates.push_back(st.kf_updates);
                msg.delta_k.push_back(st.delta_k.value_or(0.0));
                msg.theta_k_s.push_back((st.theta_k_us.value_or(0.0)) * 1e-6);
                msg.kf_drift.push_back(st.kf_drift.value_or(0.0));
                msg.kf_drift_rate.push_back(st.kf_drift_rate.value_or(0.0));
            }
        }
        msg.header.stamp    = now();
        msg.header.frame_id = "uwb";
        pub_status_->publish(msg);
    }

    std::unique_ptr<SyncEngine> engine_;
    std::mutex                  engine_mu_;

    rclcpp::Subscription<ips_msgs::msg::UwbAnchorReport>::SharedPtr sub_reports_;
    rclcpp::Subscription<ips_msgs::msg::SessionEvent>::SharedPtr    sub_events_;
    rclcpp::Publisher<ips_msgs::msg::CorrectedToA>::SharedPtr       pub_toa_;
    rclcpp::Publisher<ips_msgs::msg::SyncStatus>::SharedPtr         pub_status_;
    rclcpp::TimerBase::SharedPtr                                    status_timer_;
};

} // namespace ips

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ips::ClockSyncNode>());
    rclcpp::shutdown();
    return 0;
}
