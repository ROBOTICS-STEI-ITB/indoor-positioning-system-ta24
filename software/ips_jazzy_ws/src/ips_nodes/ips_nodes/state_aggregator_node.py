"""state_aggregator_node — JSON output sink (v3.4 — publish topik + opsi A + mapping IMU).

v3.4 (terhadap v3.3):
  * BARU: mapping sumbu IMU pada output JSON agar sama dengan file IMU Full.py:
        angular velocity:
          wx_out = -wy_in
          wy_out = -wx_in
          wz_out = -wz_in
        translational acceleration:
          ax_out =  ay_in
          ay_out =  ax_in
          az_out =  az_in
        angular acceleration:
          alphax_out = -alphay_in
          alphay_out = -alphax_in
          alphaz_out = -alphaz_in
    Catatan: angular acceleration memakai mapping yang sama dengan angular velocity
    karena merupakan turunan dari angular velocity.

v3.3 (terhadap v3.2):
  * BARU: publikasi JSON ke topik ROS `/state/json` (std_msgs/String).
    Echo dari terminal terpisah: ros2 topic echo /state/json
    Lebih bersih daripada stdout (tidak tertabrak log ROS lain).
  * Koreksi orientasi DISEDERHANAKAN ke OPSI A user:
        roll_out  = -roll_BNO
        pitch_out =  pitch_BNO
        yaw_out   = (360 - yaw_BNO_compass)  → range [0°, 360°)
    Diterapkan sebagai FLIP EULER MANUAL setelah extract ZYX dari quaternion.
    Hardcoded (tidak ada parameter).
  * Euler output dalam DERAJAT (lebih mudah dibaca daripada radian).
  * `display_mode` default 'off' karena publikasi topik adalah cara utama.

CATATAN: koreksi ini HANYA berlaku di output JSON aggregator.
Topik `/state/orientation` tetap berisi quaternion BNO mentah dari
imu_processor (TIDAK dipatch). RViz akan menampilkan orientasi BERBEDA
dari JSON. Itu disengaja — patch quaternion untuk RViz butuh diskusi
terpisah karena Euler→quat→Euler roundtrip tidak unik di ZYX.

v3.2 (recap): display_mode (off/stream/refresh).
v3.0 (recap): struktur JSON {tag_id, timestamp, position, velocity,
              acceleration, orientation}, field kosong = null.

Parameter:
  output_rate_hz   (10.0)
  log_file_path    ("")              — opsional, JSON satu-baris per snapshot
  display_mode     ("off")           — off/stream/refresh (default off)
  pretty_json      (True)
  tag_id           ("DRONE_01")
  round_decimals   (4)
  enable_imu_fields(True)
  publish_topic    ("/state/json")   — nama topik publikasi
"""

import json
import math
import sys
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Vector3Stamped,
    QuaternionStamped,
)
from std_msgs.msg import String

from ips_nodes.common import QOS_STATE_RELIABLE


# ─── ANSI escape codes ────────────────────────────────────────────────────
ANSI_CLEAR_SCREEN = "\033[2J"
ANSI_CURSOR_HOME  = "\033[H"
ANSI_HIDE_CURSOR  = "\033[?25l"
ANSI_SHOW_CURSOR  = "\033[?25h"


def _quat_to_euler_zyx_deg(qx: float, qy: float, qz: float, qw: float
                            ) -> tuple[float, float, float]:
    """Quaternion → Euler ZYX (roll, pitch, yaw) dalam DERAJAT.

    Range:
      roll  in (-180, 180]
      pitch in [-90, 90]   (gimbal lock di ±90)
      yaw   in (-180, 180]
    """
    sin_r = 2.0 * (qw * qx + qy * qz)
    cos_r = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sin_r, cos_r)
    sin_p = 2.0 * (qw * qy - qz * qx)
    if abs(sin_p) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_p)
    else:
        pitch = math.asin(sin_p)
    sin_y = 2.0 * (qw * qz + qx * qy)
    cos_y = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(sin_y, cos_y)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _apply_opsi_a(roll_zyx: float, pitch_zyx: float, yaw_zyx: float
                  ) -> tuple[float, float, float]:
    """Terapkan opsi A user ke Euler ZYX standar (semua dalam derajat).

    Opsi A user:
      roll_out  = -roll_BNO
      pitch_out =  pitch_BNO
      yaw_out   = 360 - yaw_BNO_compass

    Konversi ZYX standar ↔ BNO compass:
      yaw_BNO_compass = (-yaw_zyx) mod 360
      → yaw_out = 360 - ((-yaw_zyx) mod 360) = yaw_zyx mod 360 (di range [0, 360))

    Sign roll/pitch: konvensi ZYX standar dan BNO BNO055 sama untuk roll dan
    pitch (BNO Android orientation), jadi -roll_zyx = -roll_BNO dan pitch_zyx
    = pitch_BNO.
    """
    roll_out  = -roll_zyx
    pitch_out =  pitch_zyx
    yaw_out   = yaw_zyx % 360.0
    if yaw_out < 0:
        yaw_out += 360.0
    return roll_out, pitch_out, yaw_out


class StateAggregatorNode(Node):

    def __init__(self) -> None:
        super().__init__('state_aggregator')

        self.declare_parameter('output_rate_hz', 10.0)
        self.declare_parameter('log_file_path', '')
        self.declare_parameter('display_mode', 'off')
        self.declare_parameter('pretty_json', True)
        self.declare_parameter('tag_id', 'DRONE_01')
        self.declare_parameter('round_decimals', 4)
        self.declare_parameter('enable_imu_fields', True)
        self.declare_parameter('publish_topic', '/state/json')

        self._rate       = float(self.get_parameter('output_rate_hz').value)
        self._log_path   = self.get_parameter('log_file_path').value
        mode_param       = str(self.get_parameter('display_mode').value).lower()
        self._pretty     = bool(self.get_parameter('pretty_json').value)
        self._tag_id     = str(self.get_parameter('tag_id').value)
        self._dec        = int(self.get_parameter('round_decimals').value)
        self._enable_imu = bool(self.get_parameter('enable_imu_fields').value)
        self._pub_topic  = str(self.get_parameter('publish_topic').value)

        # Validasi & fallback display mode
        if mode_param not in ('off', 'stream', 'refresh'):
            self.get_logger().warn(
                f"display_mode='{mode_param}' tidak dikenal, pakai 'off'")
            mode_param = 'off'
        if mode_param == 'refresh' and not sys.stdout.isatty():
            self.get_logger().info(
                "stdout bukan TTY, refresh → fallback ke 'stream'")
            mode_param = 'stream'
        self._display_mode = mode_param

        # State terbaru per topik
        self._position: Optional[dict] = None
        self._lin_vel:  Optional[dict] = None
        self._ang_vel:  Optional[dict] = None
        self._lin_acc:  Optional[dict] = None
        self._ang_acc:  Optional[dict] = None
        self._orient:   Optional[dict] = None

        # Subscriptions
        self.create_subscription(
            PoseWithCovarianceStamped, '/state/position_compensated',
            self._on_position, QOS_STATE_RELIABLE)
        self.create_subscription(
            Vector3Stamped, '/state/wolf_velocity',
            self._on_linear_velocity, QOS_STATE_RELIABLE)

        if self._enable_imu:
            self.create_subscription(
                QuaternionStamped, '/state/orientation',
                self._on_orientation, QOS_STATE_RELIABLE)
            self.create_subscription(
                Vector3Stamped, '/state/angular_velocity',
                self._on_angular_velocity, QOS_STATE_RELIABLE)
            self.create_subscription(
                Vector3Stamped, '/state/translation_acceleration',
                self._on_linear_acceleration, QOS_STATE_RELIABLE)
            self.create_subscription(
                Vector3Stamped, '/state/angular_acceleration',
                self._on_angular_acceleration, QOS_STATE_RELIABLE)

        # Publisher topik JSON (UTAMA — selalu aktif)
        self._pub_json = self.create_publisher(
            String, self._pub_topic, QOS_STATE_RELIABLE)

        # File log handle (opsional)
        self._log_file = None
        if self._log_path:
            Path(self._log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(self._log_path, 'a', buffering=1)
            self.get_logger().info(f'logging JSON to {self._log_path}')

        # Refresh-mode setup
        if self._display_mode == 'refresh':
            try:
                sys.stdout.write(ANSI_HIDE_CURSOR)
                sys.stdout.flush()
            except Exception:
                pass

        self._timer = self.create_timer(1.0 / self._rate, self._emit_snapshot)

        self.get_logger().info(
            f'state_aggregator v3.4 @ {self._rate:.1f} Hz, '
            f'tag_id={self._tag_id}, publish={self._pub_topic}, '
            f'display={self._display_mode}, imu_fields={self._enable_imu}')
        self.get_logger().info(
            f'Konsumsi JSON: ros2 topic echo {self._pub_topic}')

    # ---- Helpers ------------------------------------------------------
    def _round(self, v: float) -> float:
        return round(v, self._dec)

    def _xyz(self, msg: Vector3Stamped) -> dict:
        return {
            'x': self._round(msg.vector.x),
            'y': self._round(msg.vector.y),
            'z': self._round(msg.vector.z),
        }

    def _imu_angular_xyz(self, msg: Vector3Stamped) -> dict:
        """Mapping angular velocity/acceleration sesuai IMU Full.py.

        IMU Full.py melakukan:
          gyro_x_temp = gyro_x
          gyro_x = gyro_y
          gyro_y = gyro_x_temp
          gyro_x *= -1
          gyro_y *= -1
          gyro_z *= -1

        Maka output JSON:
          x = -input_y
          y = -input_x
          z = -input_z
        """
        return {
            'x': self._round(-msg.vector.y),
            'y': self._round(-msg.vector.x),
            'z': self._round(-msg.vector.z),
        }

    def _imu_translation_accel_xyz(self, msg: Vector3Stamped) -> dict:
        """Mapping translational acceleration sesuai IMU Full.py.

        IMU Full.py hanya menukar accel_x dan accel_y, tanpa flip tanda:
          accel_x_temp = accel_x
          accel_x = accel_y
          accel_y = accel_x_temp

        Maka output JSON:
          x = input_y
          y = input_x
          z = input_z
        """
        return {
            'x': self._round(msg.vector.y),
            'y': self._round(msg.vector.x),
            'z': self._round(msg.vector.z),
        }

    # ---- Callbacks ----------------------------------------------------
    def _on_position(self, msg: PoseWithCovarianceStamped) -> None:
        self._position = {
            'x': self._round(msg.pose.pose.position.x),
            'y': self._round(msg.pose.pose.position.y),
            'z': self._round(msg.pose.pose.position.z),
        }

    def _on_linear_velocity(self, msg: Vector3Stamped) -> None:
        self._lin_vel = self._xyz(msg)

    def _on_angular_velocity(self, msg: Vector3Stamped) -> None:
        self._ang_vel = self._imu_angular_xyz(msg)

    def _on_linear_acceleration(self, msg: Vector3Stamped) -> None:
        self._lin_acc = self._imu_translation_accel_xyz(msg)

    def _on_angular_acceleration(self, msg: Vector3Stamped) -> None:
        self._ang_acc = self._imu_angular_xyz(msg)

    def _on_orientation(self, msg: QuaternionStamped) -> None:
        # 1. Extract Euler ZYX standar dari quaternion BNO mentah.
        roll_zyx, pitch_zyx, yaw_zyx = _quat_to_euler_zyx_deg(
            msg.quaternion.x, msg.quaternion.y,
            msg.quaternion.z, msg.quaternion.w)
        # 2. Apply OPSI A: flip Euler manual.
        roll_out, pitch_out, yaw_out = _apply_opsi_a(
            roll_zyx, pitch_zyx, yaw_zyx)
        self._orient = {
            'roll':  self._round(pitch_out),
            'pitch': self._round(roll_out),
            'yaw':   self._round(yaw_out),
        }

    # ---- Snapshot composition ----------------------------------------
    def _build_snapshot(self, pc_time: float) -> dict:
        def _pair(linear, angular):
            if linear is None and angular is None:
                return None
            return {'linear': linear, 'angular': angular}
        return {
            'tag_id':       self._tag_id,
            'timestamp':    round(pc_time, self._dec),
            'position':     self._position,
            'velocity':     _pair(self._lin_vel, self._ang_vel),
            'acceleration': _pair(self._lin_acc, self._ang_acc),
            'orientation':  self._orient,
        }

    def _emit_snapshot(self) -> None:
        pc_time = self.get_clock().now().nanoseconds * 1e-9
        snap = self._build_snapshot(pc_time)

        # ── Publish ke topik /state/json (UTAMA — selalu) ──
        msg = String()
        # Compact untuk topik (penerima bisa pretty-print sendiri kalau perlu)
        msg.data = json.dumps(snap)
        self._pub_json.publish(msg)

        # ── Stdout output (opsional) ──
        if self._display_mode in ('stream', 'refresh'):
            line = json.dumps(snap, indent=2) if self._pretty else json.dumps(snap)
            if self._display_mode == 'refresh':
                sys.stdout.write(ANSI_CLEAR_SCREEN + ANSI_CURSOR_HOME)
                sys.stdout.write(line + "\n")
                sys.stdout.flush()
            else:
                print(line, flush=True)

        # ── File output (selalu plain, tanpa ANSI) ──
        if self._log_file is not None:
            self._log_file.write(json.dumps(snap) + '\n')

    def destroy_node(self) -> bool:
        if self._display_mode == 'refresh':
            try:
                sys.stdout.write(ANSI_SHOW_CURSOR + "\n")
                sys.stdout.flush()
            except Exception:
                pass
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()