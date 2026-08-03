// =============================================================================
//  differentiator_node.cpp  (Tingkat 1.5 — Savitzky-Golay)
//
//    /state/position_compensated -> /state/translation_velocity
//    /state/angular_velocity     -> /state/angular_acceleration  (enable_angular)
//
//  Metode turunan dipilih via parameter `deriv_method`:
//    "backward" : beda-hingga mundur (lama; memperkuat noise)
//    "savgol"   : Savitzky-Golay causal (default; fit poly lalu turunkan,
//                 jauh lebih halus untuk sinyal noisy — port read_UDP.py)
//
//  SG memakai dt AKTUAL tiap sampel (bukan FIXED_DT 50Hz read_UDP.py), jadi
//  benar di rate IMU/posisi berapa pun. Window & orde poly via parameter.
//  Clip percepatan sudut opsional (read_UDP.py: MAX_ANGULAR_ACCEL).
// =============================================================================
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "ips_nodes_cpp/imu_filters.hpp"

namespace ips {

using std::placeholders::_1;

static rclcpp::QoS qos_state_reliable() {
    rclcpp::QoS q(20); q.reliable().keep_last(20); return q;
}

class DifferentiatorNode : public rclcpp::Node {
public:
    DifferentiatorNode() : rclcpp::Node("differentiator") {
        declare_parameter<double>("min_dt_s", 1e-4);
        declare_parameter<double>("max_dt_s", 1.0);
        declare_parameter<bool>  ("enable_angular", false);
        declare_parameter<std::string>("deriv_method", "savgol");  // "savgol"|"backward"
        declare_parameter<int>   ("sg_window", 15);
        declare_parameter<int>   ("sg_poly",   3);
        declare_parameter<double>("ang_accel_clip", 0.0);  // 0 = tanpa clip

        min_dt_         = get_parameter("min_dt_s").as_double();
        max_dt_         = get_parameter("max_dt_s").as_double();
        enable_angular_ = get_parameter("enable_angular").as_bool();
        method_         = get_parameter("deriv_method").as_string();
        const int win   = get_parameter("sg_window").as_int();
        const int poly  = get_parameter("sg_poly").as_int();
        const double clip = get_parameter("ang_accel_clip").as_double();

        use_sg_ = (method_ == "savgol");

        // SG instances: 3 utk velocity (pos), 3 utk angular accel
        for (int i = 0; i < 3; ++i) {
            sg_vel_[i]   = SavGolDerivative(static_cast<std::size_t>(win), poly, 0.0);
            sg_alpha_[i] = SavGolDerivative(static_cast<std::size_t>(win), poly, clip);
        }

        sub_pos_ = create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/state/position_compensated", qos_state_reliable(),
            std::bind(&DifferentiatorNode::on_position, this, _1));
        pub_vel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/state/translation_velocity", qos_state_reliable());

        if (enable_angular_) {
            sub_omega_ = create_subscription<
                geometry_msgs::msg::Vector3Stamped>(
                "/state/angular_velocity", qos_state_reliable(),
                std::bind(&DifferentiatorNode::on_angular_velocity, this, _1));
            pub_alpha_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "/state/angular_acceleration", qos_state_reliable());
        }

        RCLCPP_INFO(get_logger(),
            "differentiator (C++) ready — method=%s angular=%s (sg win=%d poly=%d)",
            method_.c_str(), enable_angular_ ? "true" : "false", win, poly);
    }

private:
    void on_position(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        const double t = stamp_to_sec(msg->header.stamp);
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double z = msg->pose.pose.position.z;

        if (have_prev_pos_) {
            const double dt = t - pt_;
            if (dt >= min_dt_ && dt <= max_dt_) {
                double vx, vy, vz;
                if (use_sg_) {
                    vx = sg_vel_[0].update(x, dt);
                    vy = sg_vel_[1].update(y, dt);
                    vz = sg_vel_[2].update(z, dt);
                    if (!sg_vel_[0].ready()) { pt_=t; px_=x; py_=y; pz_=z; return; }
                } else {
                    vx = (x - px_) / dt; vy = (y - py_) / dt; vz = (z - pz_) / dt;
                }
                publish_vec(pub_vel_, msg->header.stamp, msg->header.frame_id, vx, vy, vz);
            }
        } else if (use_sg_) {
            sg_vel_[0].update(x, 0.05);  // seed window (dt nominal, hasil diabaikan)
            sg_vel_[1].update(y, 0.05);
            sg_vel_[2].update(z, 0.05);
        }
        pt_ = t; px_ = x; py_ = y; pz_ = z;
        have_prev_pos_ = true;
    }

    void on_angular_velocity(
        const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        const double t  = stamp_to_sec(msg->header.stamp);
        const double wx = msg->vector.x, wy = msg->vector.y, wz = msg->vector.z;

        if (have_prev_omega_) {
            const double dt = t - ot_;
            if (dt >= min_dt_ && dt <= max_dt_) {
                double ax, ay, az;
                if (use_sg_) {
                    ax = sg_alpha_[0].update(wx, dt);
                    ay = sg_alpha_[1].update(wy, dt);
                    az = sg_alpha_[2].update(wz, dt);
                    if (!sg_alpha_[0].ready()) { ot_=t; owx_=wx; owy_=wy; owz_=wz; return; }
                } else {
                    ax = (wx - owx_) / dt; ay = (wy - owy_) / dt; az = (wz - owz_) / dt;
                }
                publish_vec(pub_alpha_, msg->header.stamp, msg->header.frame_id, ax, ay, az);
            }
        } else if (use_sg_) {
            sg_alpha_[0].update(wx, 0.05);
            sg_alpha_[1].update(wy, 0.05);
            sg_alpha_[2].update(wz, 0.05);
        }
        ot_ = t; owx_ = wx; owy_ = wy; owz_ = wz;
        have_prev_omega_ = true;
    }

    static void publish_vec(
        const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr& pub,
        const builtin_interfaces::msg::Time& stamp,
        const std::string& frame_id, double x, double y, double z) {
        geometry_msgs::msg::Vector3Stamped out;
        out.header.stamp = stamp; out.header.frame_id = frame_id;
        out.vector.x = x; out.vector.y = y; out.vector.z = z;
        pub->publish(out);
    }

    static double stamp_to_sec(const builtin_interfaces::msg::Time& s) {
        return static_cast<double>(s.sec) + static_cast<double>(s.nanosec) * 1e-9;
    }

    double min_dt_ = 1e-4, max_dt_ = 1.0;
    bool   enable_angular_ = false, use_sg_ = true;
    std::string method_ = "savgol";

    SavGolDerivative sg_vel_[3], sg_alpha_[3];

    bool   have_prev_pos_ = false;
    double pt_=0, px_=0, py_=0, pz_=0;
    bool   have_prev_omega_ = false;
    double ot_=0, owx_=0, owy_=0, owz_=0;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr            sub_omega_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr               pub_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr               pub_alpha_;
};

}  // namespace ips

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ips::DifferentiatorNode>());
    rclcpp::shutdown();
    return 0;
}
