"""udp_gateway_node — entry point of the IPS pipeline.

Replaces udp_to_csv_router.py: instead of writing CSV files, parses the
same CSV-string UDP packets and republishes them as ROS topics.

UDP input formats (from anchor + tag firmware):
    "MASTER_CLOCK,<ma_id>,<session_id>,<seq>,<tx_hex>"
    "<sa_id>,MASTER,<ma_id>,<seq>,<tk_hex>,<rx_hex>"
    "<sa_id>,TAG,<tag_id>,<seq>,NA,<rx_hex>"
    "RESET,MA,<ma_id>,<session_id>,<reason>"
    "HELLO,MA,<ma_id>,<session_id>,<info>"
    "HELLO,SA,<sa_id>,<info>"
    "HELLO,TAG,<tag_id>,<ip>"
    "HB,MA,<ma_id>,<session_id>,<n>,<m>"
    "$,<seq>,<ms>,<blink>,YAW,PITCH,ROLL,QW,QX,QY,QZ,GX,GY,GZ,AX,AY,AZ"  (IMU)
    "STAT,TAG,<id>,<blink_ok>,<blink_to>,<imu_sent>,<rssi>,<calib>"      (diagnostic)

Outputs:
    /uwb/anchor_reports  (ips_msgs/UwbAnchorReport,  BEST_EFFORT)
    /uwb/session_events  (ips_msgs/SessionEvent,     RELIABLE)
    /imu/raw             (ips_msgs/ImuTelemetry,     BEST_EFFORT)   [Tingkat 1]

Catatan Tingkat 1: data IMU di-publish APA ADANYA (tanpa konversi unit/rotasi).
Gateway hanya I/O — parsing + publish, tidak ada pengolahan. Field `blink`
dibawa lewat untuk time-alignment IMU<->posisi UWB di tahap berikutnya.
"""

import re
import socket
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from ips_msgs.msg import UwbAnchorReport, SessionEvent, ImuTelemetry

from ips_nodes.common import (
    QOS_SENSOR_BEST_EFFORT_DEEP,
    QOS_SENSOR_BEST_EFFORT,
    QOS_STATE_RELIABLE,
    DEFAULT_UDP_PORT,
    REPORT_TYPE_MASTER_CLOCK,
    REPORT_TYPE_SLAVE_MASTER,
    REPORT_TYPE_SLAVE_TAG,
    EVENT_HELLO,
    EVENT_RESET,
    EVENT_HEARTBEAT,
    EVENT_AUTO_RESTART,
)


# ---------------------------------------------------------------------------
# Regex patterns (mirrors udp_to_csv_router.py + sync_engine.py parsers)
# ---------------------------------------------------------------------------

MASTER_CSV_RE = re.compile(
    r"^MASTER_CLOCK\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*([0-9A-Fa-f]+)\s*$"
)
SLAVE_MASTER_RE = re.compile(
    r"^(\d+)\s*,\s*MASTER\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*([0-9A-Fa-f]+)\s*,\s*([0-9A-Fa-f]+)\s*$"
)
SLAVE_TAG_RE = re.compile(
    r"^(\d+)\s*,\s*TAG\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*NA\s*,\s*([0-9A-Fa-f]+)\s*$"
)
RESET_RE    = re.compile(r"^RESET\s*,\s*MA\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(.+)$")
HELLO_MA_RE = re.compile(r"^HELLO\s*,\s*MA\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(.+)$")
HELLO_SA_RE = re.compile(r"^HELLO\s*,\s*SA\s*,\s*(\d+)\s*,\s*(.+)$")
HELLO_TAG_RE = re.compile(r"^HELLO\s*,\s*TAG\s*,\s*(\d+)\s*,\s*(.+)$")
HB_RE       = re.compile(r"^HB\s*,\s*MA\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)$")

# IMU packet: $,seq,ms,blink, YAW,PITCH,ROLL, QW,QX,QY,QZ, GX,GY,GZ, AX,AY,AZ
#   3 unsigned ints (seq, ms, blink) + 13 floats (3 euler, 4 quat, 3 gyro, 3 accel)
_F = r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"   # float (boleh negatif/desimal/eksponen)
IMU_RE = re.compile(
    r"^\$\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)"          # seq, ms, blink
    + (r"\s*,\s*" + _F) * 13                            # 13 nilai float
    + r"\s*$"
)
# STAT (diagnostic) — di-parse agar tidak masuk 'ignored', tapi tidak dipublish.
STAT_TAG_RE = re.compile(r"^STAT\s*,\s*TAG\s*,\s*(\d+)\s*,(.+)$")


class UdpGatewayNode(Node):
    """UDP listener that demultiplexes anchor packets onto ROS topics."""

    def __init__(self) -> None:
        super().__init__('udp_gateway')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('udp_port', DEFAULT_UDP_PORT)
        self.declare_parameter('udp_bind_address', '0.0.0.0')
        self.declare_parameter('socket_buffer_bytes', 1 << 20)
        self.declare_parameter('print_stats_every_s', 5.0)

        self._port: int = int(self.get_parameter('udp_port').value)
        self._bind_addr: str = self.get_parameter('udp_bind_address').value
        self._sock_buf: int = int(self.get_parameter('socket_buffer_bytes').value)
        self._stats_period: float = float(self.get_parameter('print_stats_every_s').value)

        # ---- Publishers ------------------------------------------------
        self._pub_report = self.create_publisher(
            UwbAnchorReport, '/uwb/anchor_reports', QOS_SENSOR_BEST_EFFORT_DEEP
        )
        self._pub_event = self.create_publisher(
            SessionEvent, '/uwb/session_events', QOS_STATE_RELIABLE
        )
        self._pub_imu = self.create_publisher(
            ImuTelemetry, '/imu/raw', QOS_SENSOR_BEST_EFFORT
        )

        # ---- State -----------------------------------------------------
        self._ma_sessions: dict[int, int] = {}   # ma_id → current session_id
        self._counts = {'master': 0, 'slave_master': 0, 'slave_tag': 0,
                        'event': 0, 'imu': 0, 'ignored': 0}
        self._seen_anchors: set[str] = set()
        self._last_stats_t = time.time()

        # ---- Socket + worker thread ------------------------------------
        self._sock = self._open_socket()
        self._stop_event = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name='udp_gateway_rx', daemon=True
        )
        self._rx_thread.start()

        self.get_logger().info(
            f'udp_gateway listening on {self._bind_addr}:{self._port}'
        )

    # ------------------------------------------------------------------
    def _open_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self._sock_buf)
        except OSError:
            pass
        sock.bind((self._bind_addr, self._port))
        sock.settimeout(0.5)
        return sock

    # ------------------------------------------------------------------
    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                payload, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                self._maybe_print_stats()
                continue
            except OSError as exc:
                self.get_logger().error(f'socket error: {exc}')
                break

            try:
                text = payload.decode('utf-8', errors='ignore')
            except Exception:
                continue

            pc_time = time.time()
            src_ip = addr[0]
            for raw_line in text.splitlines():
                line = raw_line.strip()
                if not line:
                    continue
                self._dispatch(line, src_ip, pc_time)

            self._maybe_print_stats()

    # ------------------------------------------------------------------
    def _dispatch(self, line: str, src_ip: str, pc_time: float) -> None:
        # IMU packet first — paling sering (20Hz) & prefix '$' khas, cek murah.
        if line.startswith('$'):
            m = IMU_RE.match(line)
            if m:
                self._publish_imu(m, pc_time)
                self._counts['imu'] += 1
            else:
                self._counts['ignored'] += 1
            return

        # STAT diagnostic — diakui (tidak dipublish), agar tidak masuk 'ignored'.
        m = STAT_TAG_RE.match(line)
        if m:
            self._mark_seen(f'TAG{int(m.group(1))}@{src_ip}')
            self._counts['event'] += 1
            return

        # Control packets first (highest priority — order matters: RESET
        # must publish session event before any data report processing).
        m = RESET_RE.match(line)
        if m:
            ma_id, sess, reason = int(m.group(1)), int(m.group(2)), m.group(3)
            self._publish_session_event(EVENT_RESET, ma_id, sess, reason)
            self._update_session_with_detection(ma_id, sess, source='RESET')
            self._counts['event'] += 1
            return

        m = HELLO_MA_RE.match(line)
        if m:
            ma_id, sess = int(m.group(1)), int(m.group(2))
            self._publish_session_event(EVENT_HELLO, ma_id, sess, m.group(3))
            self._update_session_with_detection(ma_id, sess, source='HELLO')
            self._mark_seen(f'MA{ma_id}@{src_ip}')
            self._counts['event'] += 1
            return

        m = HB_RE.match(line)
        if m:
            ma_id, sess = int(m.group(1)), int(m.group(2))
            self._publish_session_event(EVENT_HEARTBEAT, ma_id, sess, '')
            self._update_session_with_detection(ma_id, sess, source='HB')
            self._counts['event'] += 1
            return

        m = HELLO_SA_RE.match(line)
        if m:
            sa_id = int(m.group(1))
            self._publish_session_event(EVENT_HELLO, sa_id, 0, m.group(2))
            self._mark_seen(f'SA{sa_id}@{src_ip}')
            self._counts['event'] += 1
            return

        m = HELLO_TAG_RE.match(line)
        if m:
            tag_id = int(m.group(1))
            self._publish_session_event(EVENT_HELLO, tag_id, 0, m.group(2))
            self._mark_seen(f'TAG{tag_id}@{src_ip}')
            self._counts['event'] += 1
            return

        # Data packets
        m = MASTER_CSV_RE.match(line)
        if m:
            self._publish_report(
                report_type=REPORT_TYPE_MASTER_CLOCK,
                reporter_id=int(m.group(1)),
                session_id=int(m.group(2)),
                source_id=int(m.group(1)),       # MA broadcasts itself
                seq=int(m.group(3)),
                tx_hex=m.group(4).upper(),
                rx_hex='',
                pc_time=pc_time, src_ip=src_ip,
            )
            self._counts['master'] += 1
            return

        m = SLAVE_MASTER_RE.match(line)
        if m:
            self._publish_report(
                report_type=REPORT_TYPE_SLAVE_MASTER,
                reporter_id=int(m.group(1)),
                session_id=0,
                source_id=int(m.group(2)),
                seq=int(m.group(3)),
                tx_hex=m.group(4).upper(),
                rx_hex=m.group(5).upper(),
                pc_time=pc_time, src_ip=src_ip,
            )
            self._mark_seen(f'SA{m.group(1)}@{src_ip}')
            self._counts['slave_master'] += 1
            return

        m = SLAVE_TAG_RE.match(line)
        if m:
            self._publish_report(
                report_type=REPORT_TYPE_SLAVE_TAG,
                reporter_id=int(m.group(1)),
                session_id=0,
                source_id=int(m.group(2)),
                seq=int(m.group(3)),
                tx_hex='',
                rx_hex=m.group(4).upper(),
                pc_time=pc_time, src_ip=src_ip,
            )
            self._mark_seen(f'SA{m.group(1)}@{src_ip}')
            self._counts['slave_tag'] += 1
            return

        # Unknown
        self._counts['ignored'] += 1

    # ------------------------------------------------------------------
    def _publish_report(
        self,
        report_type: int,
        reporter_id: int,
        session_id: int,
        source_id: int,
        seq: int,
        tx_hex: str,
        rx_hex: str,
        pc_time: float,
        src_ip: str,
    ) -> None:
        msg = UwbAnchorReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uwb'
        msg.report_type = report_type
        msg.reporter_id = reporter_id
        msg.session_id = session_id
        msg.source_id = source_id
        msg.seq = seq
        msg.tx_hex = tx_hex
        msg.rx_hex = rx_hex
        msg.pc_time_s = pc_time
        msg.src_ip = src_ip
        self._pub_report.publish(msg)

    def _publish_session_event(
        self, kind: int, device_id: int, session_id: int, reason: str
    ) -> None:
        msg = SessionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uwb'
        msg.kind = kind
        msg.device_id = device_id
        msg.session_id = session_id
        msg.reason = reason
        self._pub_event.publish(msg)

    # ------------------------------------------------------------------
    def _publish_imu(self, m: 're.Match', pc_time: float) -> None:
        """Publish paket IMU APA ADANYA. Groups (regex IMU_RE):
        1=seq 2=ms 3=blink 4=yaw 5=pitch 6=roll 7=qw 8=qx 9=qy 10=qz
        11=gx 12=gy 13=gz 14=ax 15=ay 16=az
        """
        msg = ImuTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tag_imu'
        # Tag ID tidak ada di paket IMU (cuma di STAT/HELLO) → 0 sbg placeholder.
        msg.tag_id      = 0
        msg.seq         = int(m.group(1)) & 0xFFFFFFFF
        msg.tag_time_ms = int(m.group(2)) & 0xFFFFFFFF
        msg.blink       = int(m.group(3)) & 0xFFFFFFFF

        msg.yaw_deg   = float(m.group(4))
        msg.pitch_deg = float(m.group(5))
        msg.roll_deg  = float(m.group(6))

        msg.quat_w = float(m.group(7))
        msg.quat_x = float(m.group(8))
        msg.quat_y = float(m.group(9))
        msg.quat_z = float(m.group(10))

        gyro = Vector3()
        gyro.x = float(m.group(11))
        gyro.y = float(m.group(12))
        gyro.z = float(m.group(13))
        msg.angular_velocity = gyro

        accel = Vector3()
        accel.x = float(m.group(14))
        accel.y = float(m.group(15))
        accel.z = float(m.group(16))
        msg.linear_acceleration = accel

        self._pub_imu.publish(msg)

    # ------------------------------------------------------------------
    def _update_session_with_detection(
        self, ma_id: int, sess: int, source: str
    ) -> None:
        """Detect MA restart by session_id change and emit auto-event."""
        prev = self._ma_sessions.get(ma_id)
        if prev is not None and prev != sess:
            self.get_logger().warning(
                f'MA{ma_id} session change ({source}): {prev} → {sess}'
            )
            self._publish_session_event(
                EVENT_AUTO_RESTART, ma_id, sess,
                f'session changed via {source}: {prev}->{sess}'
            )
        self._ma_sessions[ma_id] = sess

    def _mark_seen(self, tag: str) -> None:
        if tag not in self._seen_anchors:
            self._seen_anchors.add(tag)
            self.get_logger().info(f'[new] {tag}')

    def _maybe_print_stats(self) -> None:
        now = time.time()
        if now - self._last_stats_t < self._stats_period:
            return
        self._last_stats_t = now
        c = self._counts
        self.get_logger().info(
            f'stats: master={c["master"]} '
            f'sa_master={c["slave_master"]} sa_tag={c["slave_tag"]} '
            f'imu={c["imu"]} '
            f'events={c["event"]} ignored={c["ignored"]} '
            f'anchors={sorted(self._seen_anchors)}'
        )

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        self._stop_event.set()
        try:
            self._rx_thread.join(timeout=2.0)
        finally:
            try:
                self._sock.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
