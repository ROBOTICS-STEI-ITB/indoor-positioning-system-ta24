"""calibration_service_node — config provider.

Loads anchor positions from YAML and publishes them on a latched
(TRANSIENT_LOCAL) topic so any node that joins later still sees them.
Also hosts SetAnchorConfig service for runtime updates.

Outputs (latched):
    /uwb/anchor_config   (geometry_msgs/PoseArray)

Service:
    ~/set_anchor_config  (ips_msgs/srv/SetAnchorConfig)
"""

from pathlib import Path
from typing import Any

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

from ips_msgs.srv import SetAnchorConfig

from ips_nodes.common import QOS_LATCHED_CONFIG


class CalibrationServiceNode(Node):
    """Loads YAML calibration and exposes it via a latched topic + service."""

    def __init__(self) -> None:
        super().__init__('calibration_service')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('anchors_yaml', '')

        anchors_path: str = self.get_parameter('anchors_yaml').value

        # ---- Latched publisher ----------------------------------------
        self._pub_anchors = self.create_publisher(
            PoseArray, '/uwb/anchor_config', QOS_LATCHED_CONFIG
        )

        # ---- Service --------------------------------------------------
        self._srv = self.create_service(
            SetAnchorConfig,
            '~/set_anchor_config',
            self._on_set_anchor_config,
        )

        # ---- Load + publish (one-shot, latched thereafter) ------------
        if anchors_path:
            self._load_and_publish_anchors(anchors_path)
        else:
            self.get_logger().warning(
                'anchors_yaml param not set — no anchor_config published; '
                'use the SetAnchorConfig service to provide one'
            )

        self.get_logger().info('calibration_service ready')

    # ------------------------------------------------------------------
    def _load_and_publish_anchors(self, path: str) -> None:
        try:
            data = self._read_yaml(path)
        except OSError as exc:
            self.get_logger().error(f'cannot read anchors yaml {path}: {exc}')
            return

        anchors = data.get('anchors', [])
        if not anchors:
            self.get_logger().error(f'{path}: no "anchors" entries found')
            return

        ids = [int(a['id']) for a in anchors]
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Encode IDs in frame_id as comma-separated string (avoids needing
        # a custom message just for config).
        msg.header.frame_id = ','.join(str(i) for i in ids)
        for a in anchors:
            p = Pose()
            p.position.x = float(a['x'])
            p.position.y = float(a['y'])
            p.position.z = float(a['z'])
            p.orientation.w = 1.0
            msg.poses.append(p)
        self._pub_anchors.publish(msg)
        self.get_logger().info(
            f'published anchor_config for ids={ids} ({len(ids)} anchors)'
        )

    @staticmethod
    def _read_yaml(path: str) -> dict[str, Any]:
        with open(Path(path), 'r') as f:
            return yaml.safe_load(f) or {}

    # ------------------------------------------------------------------
    def _on_set_anchor_config(
        self,
        request: SetAnchorConfig.Request,
        response: SetAnchorConfig.Response,
    ) -> SetAnchorConfig.Response:
        ids = list(request.anchor_ids)
        poses = list(request.anchor_poses.poses)
        if len(ids) != len(poses):
            response.success = False
            response.message = (
                f'length mismatch: {len(ids)} ids vs {len(poses)} poses'
            )
            return response

        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = ','.join(str(i) for i in ids)
        out.poses = poses
        self._pub_anchors.publish(out)

        response.success = True
        response.message = f'updated {len(ids)} anchors'
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrationServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
