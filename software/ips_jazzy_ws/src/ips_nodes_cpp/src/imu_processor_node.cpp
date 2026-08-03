// =============================================================================
//  imu_processor_node.cpp  (Tingkat 1.5 — filter pipeline)
//
//  Subscribe /imu/raw (ips_msgs/ImuTelemetry) -> publish:
//    /state/orientation              (geometry_msgs/QuaternionStamped)
//    /state/angular_velocity         (geometry_msgs/Vector3Stamped)  [filtered]
//    /state/translation_acceleration (geometry_msgs/Vector3Stamped)  [filtered]
//
//  Pipeline filter (port dari read_UDP.py, tanpa notch — lihat catatan):
//    gyro  : raw -> LPF(EMA) -> Kalman1D   (per sumbu)
//    accel : raw -> LPF(EMA) -> Kalman1D   (per sumbu, opsional)
//  Semua filter DEFAULT OFF (apply_*=false) -> passthrough apa adanya.
//  Aktifkan via parameter saat butuh smoothing.
//
//  CATATAN notch: notch 12Hz dari read_UDP.py TIDAK disertakan — ia spesifik
//  ke frekuensi getaran propeller tertentu & di-tuning untuk 50Hz. Memakainya
//  tanpa verifikasi puncak 12Hz + penyesuaian ke 20Hz berisiko membuang sinyal
//  valid. Tambahkan terpisah setelah analisis spektrum bila perlu.
//
//  Orientasi: quaternion native BNO055 langsung (tidak difilter — sudah hasil
//  fusi chip; memfilter quaternion butuh penanganan khusus, bukan EMA skalar).
// =============================================================================
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "ips_msgs/msg/imu_telemetry.hpp"

#include "ips_nodes_cpp/imu_filters.hpp"

namespace ips {

using std::placeholders::_1;

static rclcpp::QoS qos_sensor_best_effort() {
    rclcpp::QoS q(50); q.best_effort().keep_last(50); return q;
}
static rclcpp::QoS qos_state_reliable() {
    rclcpp::QoS q(20); q.reliable().keep_last(20); return q;
}

class ImuProcessorNode : public rclcpp::Node {
public:
    ImuProcessorNode() : rclcpp::Node("imu_processor") {
        // ---- Parameters: gyro filter ----
        declare_parameter<bool>  ("gyro_apply_lpf",    false);
        declare_parameter<double>("gyro_lpf_alpha",    0.05);
        declare_parameter<bool>  ("gyro_apply_kalman", false);
        declare_parameter<double>("gyro_kf_q",         0.0005);
        declare_parameter<double>("gyro_kf_r",         0.05);
        // ---- Parameters: accel filter ----
        declare_parameter<bool>  ("accel_apply_lpf",    false);
        declare_parameter<double>("accel_lpf_alpha",    0.05);
        declare_parameter<bool>  ("accel_apply_kalman", false);
        declare_parameter<double>("accel_kf_q",         0.0005);
        declare_parameter<double>("accel_kf_r",         0.05);
        declare_parameter<std::string>("frame_id", "tag_imu");

        gyro_lpf_   = get_parameter("gyro_apply_lpf").as_bool();
        gyro_kf_    = get_parameter("gyro_apply_kalman").as_bool();
        accel_lpf_  = get_parameter("accel_apply_lpf").as_bool();
        accel_kf_   = get_parameter("accel_apply_kalman").as_bool();
        frame_id_   = get_parameter("frame_id").as_string();

        const double ga = get_parameter("gyro_lpf_alpha").as_double();
        const double gq = get_parameter("gyro_kf_q").as_double();
        const double gr = get_parameter("gyro_kf_r").as_double();
        const double aa = get_parameter("accel_lpf_alpha").as_double();
        const double aq = get_parameter("accel_kf_q").as_double();
        const double ar = get_parameter("accel_kf_r").as_double();

        for (int i = 0; i < 3; ++i) {
            g_lpf_[i].set_alpha(ga);  g_kf_[i].set_params(gq, gr);
            a_lpf_[i].set_alpha(aa);  a_kf_[i].set_params(aq, ar);
        }

        sub_ = create_subscription<ips_msgs::msg::ImuTelemetry>(
            "/imu/raw", qos_sensor_best_effort(),
            std::bind(&ImuProcessorNode::on_imu, this, _1));

        pub_orient_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "/state/orientation", qos_state_reliable());
        pub_omega_  = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/state/angular_velocity", qos_state_reliable());
        pub_accel_  = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/state/translation_acceleration", qos_state_reliable());

        RCLCPP_INFO(get_logger(),
            "imu_processor (C++) ready — gyro[lpf=%s kf=%s] accel[lpf=%s kf=%s]",
            gyro_lpf_ ? "on" : "off", gyro_kf_ ? "on" : "off",
            accel_lpf_ ? "on" : "off", accel_kf_ ? "on" : "off");
    }

private:
    void on_imu(const ips_msgs::msg::ImuTelemetry::SharedPtr msg) {
        // ---- Orientation: quaternion native (passthrough) ----
        geometry_msgs::msg::QuaternionStamped q;
        q.header.stamp = msg->header.stamp;
        q.header.frame_id = frame_id_;
        q.quaternion.w = msg->quat_w; q.quaternion.x = msg->quat_x;
        q.quaternion.y = msg->quat_y; q.quaternion.z = msg->quat_z;
        pub_orient_->publish(q);

        // ---- Gyro: raw -> [LPF] -> [Kalman] ----
        double g[3] = {msg->angular_velocity.x,
                       msg->angular_velocity.y,
                       msg->angular_velocity.z};
        for (int i = 0; i < 3; ++i) {
            if (gyro_lpf_) g[i] = g_lpf_[i].update(g[i]);
            if (gyro_kf_)  g[i] = g_kf_[i].update(g[i]);
        }
        geometry_msgs::msg::Vector3Stamped w;
        w.header.stamp = msg->header.stamp;
        w.header.frame_id = frame_id_;
        w.vector.x = g[0]; w.vector.y = g[1]; w.vector.z = g[2];
        pub_omega_->publish(w);

        // ---- Accel: raw -> [LPF] -> [Kalman] ----
        double a[3] = {msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z};
        for (int i = 0; i < 3; ++i) {
            if (accel_lpf_) a[i] = a_lpf_[i].update(a[i]);
            if (accel_kf_)  a[i] = a_kf_[i].update(a[i]);
        }
        geometry_msgs::msg::Vector3Stamped acc;
        acc.header.stamp = msg->header.stamp;
        acc.header.frame_id = frame_id_;
        acc.vector.x = a[0]; acc.vector.y = a[1]; acc.vector.z = a[2];
        pub_accel_->publish(acc);
    }

    bool gyro_lpf_ = false, gyro_kf_ = false;
    bool accel_lpf_ = false, accel_kf_ = false;
    std::string frame_id_ = "tag_imu";

    LowPassEma     g_lpf_[3], a_lpf_[3];
    KalmanFilter1D g_kf_[3],  a_kf_[3];

    rclcpp::Subscription<ips_msgs::msg::ImuTelemetry>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_orient_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr    pub_omega_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr    pub_accel_;
};

}  // namespace ips

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ips::ImuProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
