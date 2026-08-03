"""differentiator_node — backward-difference derivative.

At this stage of the project (no IMU yet), only translation velocity is
computed (from /state/position). Once the tag IMU is online and the
imu_processor publishes /state/angular_velocity, the angular_acceleration
branch automatically becomes active — set `enable_angular` to true.

Inputs:
    /state/position           (geometry_msgs/PoseWithCovarianceStamped, RELIABLE)
    /state/angular_velocity   (geometry_msgs/Vector3Stamped, RELIABLE) [optional]

Outputs:
    /state/translation_velocity (geometry_msgs/Vector3Stamped, RELIABLE)
    /state/angular_acceleration (geometry_msgs/Vector3Stamped, RELIABLE) [optional]

Note: backward difference amplifies noise. The position KF inside
position_solver already contains a velocity state, so a future
improvement is to publish that velocity directly and skip this node
for translation_velocity.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Vector3Stamped,
)

from ips_nodes.common import QOS_STATE_RELIABLE


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class DifferentiatorNode(Node):
    """Computes time-derivatives of state streams."""

    def __init__(self) -> None:
        super().__init__('differentiator')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('min_dt_s', 1e-4)
        self.declare_parameter('max_dt_s', 1.0)
        # IMU-side branch (off until tag IMU is integrated)
        self.declare_parameter('enable_angular', False)

        self._min_dt: float = float(self.get_parameter('min_dt_s').value)
        self._max_dt: float = float(self.get_parameter('max_dt_s').value)
        self._enable_angular: bool = bool(self.get_parameter('enable_angular').value)

        # ---- Position branch (always on) ------------------------------
        self._sub_pos = self.create_subscription(
            PoseWithCovarianceStamped,
            '/state/position_compensated',
            self._on_position,
            QOS_STATE_RELIABLE,
        )
        self._pub_vel = self.create_publisher(
            Vector3Stamped,
            '/state/translation_velocity',
            QOS_STATE_RELIABLE,
        )
        self._prev_pos: Optional[tuple[float, float, float, float]] = None

        # ---- Angular branch (optional) --------------------------------
        self._prev_omega: Optional[tuple[float, float, float, float]] = None
        if self._enable_angular:
            self._sub_omega = self.create_subscription(
                Vector3Stamped,
                '/state/angular_velocity',
                self._on_angular_velocity,
                QOS_STATE_RELIABLE,
            )
            self._pub_alpha = self.create_publisher(
                Vector3Stamped,
                '/state/angular_acceleration',
                QOS_STATE_RELIABLE,
            )

        self.get_logger().info(
            f'differentiator ready (angular={self._enable_angular})'
        )

    # ------------------------------------------------------------------
    def _on_position(self, msg: PoseWithCovarianceStamped) -> None:
        t = _stamp_to_seconds(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if self._prev_pos is not None:
            t0, x0, y0, z0 = self._prev_pos
            dt = t - t0
            if self._min_dt <= dt <= self._max_dt:
                vx = (x - x0) / dt
                vy = (y - y0) / dt
                vz = (z - z0) / dt
                self._publish_vector(
                    self._pub_vel,
                    msg.header.stamp,
                    msg.header.frame_id,
                    vx, vy, vz,
                )
        self._prev_pos = (t, x, y, z)

    def _on_angular_velocity(self, msg: Vector3Stamped) -> None:
        t = _stamp_to_seconds(msg.header.stamp)
        wx, wy, wz = msg.vector.x, msg.vector.y, msg.vector.z

        if self._prev_omega is not None:
            t0, wx0, wy0, wz0 = self._prev_omega
            dt = t - t0
            if self._min_dt <= dt <= self._max_dt:
                ax = (wx - wx0) / dt
                ay = (wy - wy0) / dt
                az = (wz - wz0) / dt
                self._publish_vector(
                    self._pub_alpha,
                    msg.header.stamp,
                    msg.header.frame_id,
                    ax, ay, az,
                )
        self._prev_omega = (t, wx, wy, wz)

    # ------------------------------------------------------------------
    @staticmethod
    def _publish_vector(pub, stamp, frame_id, x, y, z) -> None:
        out = Vector3Stamped()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        out.vector.x = float(x)
        out.vector.y = float(y)
        out.vector.z = float(z)
        pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DifferentiatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
