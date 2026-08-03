#!/usr/bin/env python3
"""optitrack_bridge_node — terima streaming OptiTrack NatNet, publish GT ke ROS 2.

Self-contained: TIDAK butuh natnet_ros2, vrpn, atau SDK eksternal.
Hanya parsing rigid body dari NatNet protocol (v3.0+/v4.0+).

Alur:
  1. (opsional) Kirim NAT_CONNECT ke command port → terima server info (versi)
  2. Join multicast (atau unicast) data stream
  3. Parse NAT_FRAMEOFDATA → ekstrak rigid body pose
  4. Transform OptiTrack (Y-up) → koordinat sistem (Z-up, tangan-kanan)
  5. Publish /gt/pose (PoseStamped) + /gt/position (PointStamped)

Transform (IDENTIK export_gt_synced.py, tapi NatNet kirim METER bukan mm):
    x_sys = x_opti − X_OPTI_SA2
    y_sys = Z_OPTI_SA2 − z_opti
    z_sys = y_opti − Z_FLOOR

Setup jaringan:
  - PC Motive + laptop ROS di subnet sama (mis. 192.168.10.0/24)
  - Motive → Data Streaming → Broadcast Frame ON, Local Interface = IP PC Motive
  - Multicast default 239.255.42.99:1511 (atau unicast ke IP laptop)
  - Firewall laptop: buka port 1510-1511/udp
  - Rigid body sudah dibuat di Motive dari marker tag

Pakai:
  ros2 run ips_nodes optitrack_bridge
  ros2 run ips_nodes optitrack_bridge --ros-args \\
      -p server_ip:=192.168.10.101 \\
      -p rigid_body_id:=1
"""

import socket
import struct
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

# ─── NatNet constants ────────────────────────────────────────────────────────
NAT_CONNECT        = 0
NAT_SERVERINFO     = 1
NAT_FRAMEOFDATA    = 7
NAT_PING           = 13      # NatNet 4.1+

MULTICAST_DEFAULT  = "239.255.42.99"
DATA_PORT_DEFAULT  = 1511
CMD_PORT_DEFAULT   = 1510

SOCKET_BUFSIZE     = 65536


# ─── NatNet packet parser ────────────────────────────────────────────────────

def _unpack_cstr(data: bytes, offset: int) -> tuple[str, int]:
    """Unpack null-terminated C string from data at offset."""
    end = data.index(b'\x00', offset)
    return data[offset:end].decode('utf-8', errors='replace'), end + 1


def _parse_rigid_bodies(data: bytes, offset: int, natnet_major: int) -> tuple[list, int]:
    """Parse rigid body section. Returns list of (id, pos, quat, valid)."""
    n_bodies, = struct.unpack_from('<i', data, offset); offset += 4
    bodies = []
    for _ in range(n_bodies):
        rb_id, = struct.unpack_from('<i', data, offset); offset += 4
        px, py, pz = struct.unpack_from('<3f', data, offset); offset += 12
        qx, qy, qz, qw = struct.unpack_from('<4f', data, offset); offset += 16

        if natnet_major >= 3:
            # v3.0+: int16 params (bit 0 = tracking valid), no per-body markers
            params, = struct.unpack_from('<h', data, offset); offset += 2
            valid = (params & 0x01) != 0
        else:
            # v2.x: per-body markers + mean error
            n_mrk, = struct.unpack_from('<i', data, offset); offset += 4
            offset += n_mrk * 12       # positions
            offset += n_mrk * 4        # ids
            offset += n_mrk * 4        # sizes
            _mean_err, = struct.unpack_from('<f', data, offset); offset += 4
            # v2.6+: params
            params, = struct.unpack_from('<h', data, offset); offset += 2
            valid = (params & 0x01) != 0

        bodies.append((rb_id, (px, py, pz), (qx, qy, qz, qw), valid))
    return bodies, offset


def parse_frame_data(data: bytes, natnet_major: int = 4) -> Optional[tuple[int, list]]:
    """Parse NAT_FRAMEOFDATA packet, return (frame_number, rigid_bodies) or None."""
    try:
        offset = 0
        msg_id, = struct.unpack_from('<H', data, offset); offset += 2
        _pkt_sz, = struct.unpack_from('<H', data, offset); offset += 2

        if msg_id != NAT_FRAMEOFDATA:
            return None

        frame_num, = struct.unpack_from('<i', data, offset); offset += 4

        # ── marker sets (skip) ──
        n_sets, = struct.unpack_from('<i', data, offset); offset += 4
        for _ in range(n_sets):
            _name, offset = _unpack_cstr(data, offset)
            n_mrk, = struct.unpack_from('<i', data, offset); offset += 4
            offset += n_mrk * 12  # float32[3] per marker

        # ── unlabeled markers (skip; present but usually count=0 in v4.0+) ──
        n_other, = struct.unpack_from('<i', data, offset); offset += 4
        offset += n_other * 12

        # ── rigid bodies ──
        bodies, offset = _parse_rigid_bodies(data, offset, natnet_major)

        return frame_num, bodies

    except (struct.error, ValueError, IndexError):
        return None


def parse_server_info(data: bytes) -> Optional[tuple[str, tuple, tuple]]:
    """Parse NAT_SERVERINFO response → (app_name, app_ver, natnet_ver)."""
    try:
        msg_id, = struct.unpack_from('<H', data, 0)
        if msg_id != NAT_SERVERINFO:
            return None
        app_name = data[4:260].split(b'\x00')[0].decode('utf-8', errors='replace')
        app_ver = struct.unpack_from('4B', data, 260)
        nn_ver = struct.unpack_from('4B', data, 264)
        return app_name, app_ver, nn_ver
    except (struct.error, IndexError):
        return None


# ─── ROS 2 node ──────────────────────────────────────────────────────────────

class OptiTrackBridgeNode(Node):
    """Terima OptiTrack NatNet stream, transform & publish GT."""

    def __init__(self) -> None:
        super().__init__('optitrack_bridge')

        # ── Parameters ──
        self.declare_parameter('server_ip', '192.168.10.101')
        self.declare_parameter('multicast_addr', MULTICAST_DEFAULT)
        self.declare_parameter('data_port', DATA_PORT_DEFAULT)
        self.declare_parameter('command_port', CMD_PORT_DEFAULT)
        self.declare_parameter('use_multicast', True)
        self.declare_parameter('rigid_body_id', 1)
        self.declare_parameter('frame_id', 'world')
        # Koordinat transform (IDENTIK export_gt_synced.py)
        self.declare_parameter('x_opti_sa2', 0.646)
        self.declare_parameter('z_opti_sa2', 3.425)
        self.declare_parameter('z_floor', 0.0)
        # Rate limit (0 = setiap frame; >0 = max publish Hz)
        self.declare_parameter('max_publish_hz', 0.0)
        # NatNet version fallback (jika connect gagal)
        self.declare_parameter('natnet_major', 4)

        self._server_ip = str(self.get_parameter('server_ip').value)
        self._mcast_addr = str(self.get_parameter('multicast_addr').value)
        self._data_port = int(self.get_parameter('data_port').value)
        self._cmd_port = int(self.get_parameter('command_port').value)
        self._use_mcast = bool(self.get_parameter('use_multicast').value)
        self._rb_id = int(self.get_parameter('rigid_body_id').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._x_sa2 = float(self.get_parameter('x_opti_sa2').value)
        self._z_sa2 = float(self.get_parameter('z_opti_sa2').value)
        self._z_floor = float(self.get_parameter('z_floor').value)
        self._max_hz = float(self.get_parameter('max_publish_hz').value)
        self._nn_major = int(self.get_parameter('natnet_major').value)

        self._min_interval = 1.0 / self._max_hz if self._max_hz > 0 else 0.0
        self._last_pub_t = 0.0

        # ── Publishers ──
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub_pose = self.create_publisher(PoseStamped, '/gt/pose', qos)
        self._pub_point = self.create_publisher(PointStamped, '/gt/position', qos)

        # ── Stats ──
        self._n_frames = 0
        self._n_published = 0
        self._n_invalid = 0
        self._stat_timer = self.create_timer(5.0, self._log_stats)

        # ── Connect & start ──
        self._try_get_version()
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        mode = "multicast " + self._mcast_addr if self._use_mcast else "unicast"
        self.get_logger().info(
            f'optitrack_bridge aktif — {mode}:{self._data_port}, '
            f'server={self._server_ip}, rigid_body_id={self._rb_id}, '
            f'NatNet v{self._nn_major}.x, '
            f'transform: x−{self._x_sa2:.3f}, {self._z_sa2:.3f}−z, y−{self._z_floor}')

    # ── NatNet version detection (opsional, best-effort) ──
    def _try_get_version(self) -> None:
        """Kirim NAT_CONNECT ke command port, baca server info (versi)."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2.0)
            # NAT_CONNECT: msg_id=0, pkt_size=0, payload "Ping\0" padded 256
            payload = b'Ping\x00' + b'\x00' * 251
            pkt = struct.pack('<HH', NAT_CONNECT, len(payload)) + payload
            sock.sendto(pkt, (self._server_ip, self._cmd_port))
            data, _ = sock.recvfrom(SOCKET_BUFSIZE)
            info = parse_server_info(data)
            if info:
                app, app_v, nn_v = info
                self._nn_major = nn_v[0]
                self.get_logger().info(
                    f'NatNet server: {app} v{".".join(map(str,app_v))}, '
                    f'NatNet v{".".join(map(str,nn_v))}')
            sock.close()
        except (socket.timeout, OSError) as e:
            self.get_logger().warn(
                f'gagal kontak command port {self._server_ip}:{self._cmd_port} '
                f'({e}) — pakai NatNet v{self._nn_major} (fallback). '
                f'Pastikan: (1) IP server benar, (2) firewall buka, '
                f'(3) Motive streaming aktif.')

    # ── Data receiver loop (berjalan di thread) ──
    def _recv_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass

        sock.bind(('', self._data_port))

        if self._use_mcast:
            mreq = struct.pack(
                '4s4s',
                socket.inet_aton(self._mcast_addr),
                socket.inet_aton('0.0.0.0'))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        sock.settimeout(2.0)
        self.get_logger().info(f'mendengarkan data port {self._data_port}...')

        while self._running and rclpy.ok():
            try:
                data, _ = sock.recvfrom(SOCKET_BUFSIZE)
            except socket.timeout:
                continue
            except OSError:
                break

            result = parse_frame_data(data, self._nn_major)
            if result is None:
                continue

            frame_num, bodies = result
            self._n_frames += 1

            # Cari rigid body yang diminta
            target = None
            for rb_id, pos, quat, valid in bodies:
                if rb_id == self._rb_id:
                    target = (pos, quat, valid)
                    break

            if target is None:
                continue

            pos_opti, quat_opti, valid = target
            if not valid:
                self._n_invalid += 1
                continue

            # Rate limit
            now = time.monotonic()
            if self._min_interval > 0 and (now - self._last_pub_t) < self._min_interval:
                continue
            self._last_pub_t = now

            # ── Transform OptiTrack → sistem ──
            # OptiTrack Y-up: pos = (x, y, z)_opti
            # NatNet mengirim dalam METER (bukan mm)
            x_opti, y_opti, z_opti = pos_opti
            x_sys = x_opti - self._x_sa2         # x_sys = x_opti − X_OPTI_SA2
            y_sys = self._z_sa2 - z_opti          # y_sys = Z_OPTI_SA2 − z_opti
            z_sys = y_opti - self._z_floor        # z_sys = y_opti − Z_FLOOR

            # Quaternion transform: swap Y-up → Z-up
            # OptiTrack quat (x,y,z,w)_opti → sistem (Z-up tangan-kanan)
            # Rotasi kerangka: R = Rx(−90°) → q_sys = q_frame_rot ⊗ q_opti
            # Untuk DEMO visualisasi, posisi yang penting; orientasi
            # di-passthrough (cukup akurat untuk RViz marker).
            qx_o, qy_o, qz_o, qw_o = quat_opti

            # ── Publish ──
            stamp = self.get_clock().now().to_msg()

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self._frame_id
            pose.pose.position.x = x_sys
            pose.pose.position.y = y_sys
            pose.pose.position.z = z_sys
            pose.pose.orientation.x = qx_o
            pose.pose.orientation.y = qy_o
            pose.pose.orientation.z = qz_o
            pose.pose.orientation.w = qw_o
            self._pub_pose.publish(pose)

            pt = PointStamped()
            pt.header = pose.header
            pt.point.x = x_sys
            pt.point.y = y_sys
            pt.point.z = z_sys
            self._pub_point.publish(pt)

            self._n_published += 1

        sock.close()

    # ── Logging ──
    def _log_stats(self) -> None:
        if self._n_frames == 0 and self._n_published == 0:
            self.get_logger().warn(
                f'belum terima frame — cek: (1) Motive streaming aktif? '
                f'(2) server_ip={self._server_ip} benar? '
                f'(3) firewall port {self._data_port}/udp terbuka? '
                f'(4) subnet sama?')
            return
        hz = self._n_published / 5.0 if self._n_published > 0 else 0
        self.get_logger().info(
            f'frame={self._n_frames} pub={self._n_published} ({hz:.0f} Hz) '
            f'invalid={self._n_invalid} (rb_id={self._rb_id})')
        self._n_frames = 0
        self._n_published = 0
        self._n_invalid = 0

    def destroy_node(self) -> bool:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OptiTrackBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
