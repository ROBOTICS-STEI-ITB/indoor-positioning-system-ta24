"""imu_processor_node — IMU data processing.

Subscribes to raw IMU telemetry and emits three independent state topics:
orientation (from quaternion), angular velocity, and linear acceleration.

Inputs:
    /imu/raw                       (ips_msgs/ImuTelemetry, BEST_EFFORT)

Outputs:
    /state/orientation             (geometry_msgs/QuaternionStamped, RELIABLE)
    /state/angular_velocity        (geometry_msgs/Vector3Stamped,    RELIABLE)
    /state/translation_acceleration (geometry_msgs/Vector3Stamped,   RELIABLE)

Notes:
    - BNO055 already outputs fused orientation, but a light KF / complementary
      filter on linear acceleration helps reduce drift.
    - Operates at IMU rate (~100 Hz).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped

from ips_msgs.msg import ImuTelemetry

from ips_nodes.common import (
    QOS_SENSOR_BEST_EFFORT,
    QOS_STATE_RELIABLE,
)


class ImuProcessorNode(Node):
    """Parses raw IMU and republishes individual state quantities."""

    def __init__(self) -> None:
        super().__init__('imu_processor')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('apply_kalman_filter', True)
        self.declare_parameter('frame_id', 'imu')

        self._use_kf: bool = self.get_parameter('apply_kalman_filter').value
        self._frame_id: str = self.get_parameter('frame_id').value

        # ---- Subscription -----------------------------------------------
        self._sub = self.create_subscription(
            ImuTelemetry, '/imu/raw', self._on_imu, QOS_SENSOR_BEST_EFFORT
        )

        # ---- Publishers --------------------------------------------------
        self._pub_orient = self.create_publisher(
            QuaternionStamped, '/state/orientation', QOS_STATE_RELIABLE
        )
        self._pub_omega = self.create_publisher(
            Vector3Stamped, '/state/angular_velocity', QOS_STATE_RELIABLE
        )
        self._pub_accel = self.create_publisher(
            Vector3Stamped, '/state/translation_acceleration', QOS_STATE_RELIABLE
        )

        # ---- Internal state ---------------------------------------------
        # Filter state for accel smoothing (None until first sample).
        self._accel_filter_state: dict | None = None

        self.get_logger().info('imu_processor ready')

    # ------------------------------------------------------------------
    def _on_imu(self, msg: ImuTelemetry) -> None:
        # ---- Orientation -----------------------------------------------
        qmsg = QuaternionStamped()
        qmsg.header.stamp = msg.header.stamp
        qmsg.header.frame_id = self._frame_id
        qmsg.quaternion.w = msg.quat_w
        qmsg.quaternion.x = msg.quat_x
        qmsg.quaternion.y = msg.quat_y
        qmsg.quaternion.z = msg.quat_z
        self._pub_orient.publish(qmsg)

        # ---- Angular velocity (pass-through) ---------------------------
        omega_msg = Vector3Stamped()
        omega_msg.header.stamp = msg.header.stamp
        omega_msg.header.frame_id = self._frame_id
        omega_msg.vector.x = msg.angular_velocity.x
        omega_msg.vector.y = msg.angular_velocity.y
        omega_msg.vector.z = msg.angular_velocity.z
        self._pub_omega.publish(omega_msg)

        # ---- Linear acceleration (filtered) ----------------------------
        ax, ay, az = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        if self._use_kf:
            # TODO: apply Kalman filter / low-pass / bias removal.
            # ax, ay, az = self._filter_accel(ax, ay, az)
            pass

        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = msg.header.stamp
        accel_msg.header.frame_id = self._frame_id
        accel_msg.vector.x = ax
        accel_msg.vector.y = ay
        accel_msg.vector.z = az
        self._pub_accel.publish(accel_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
