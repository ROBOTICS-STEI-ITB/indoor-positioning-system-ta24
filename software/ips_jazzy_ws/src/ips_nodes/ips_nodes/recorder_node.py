"""recorder_node — on-demand CSV recording service.

Subscribes to key topics at all times, but only writes to CSV files
while recording is active. Controlled via the ~/control service.

Recorded layers (14 CSV files per session):
    1. position.csv         — /state/position (KF-smoothed, ~5 Hz)
    2. position_chan.csv     — /state/position_chan (raw Chan, ~5 Hz)
    3. corrected_toa.csv    — /uwb/corrected_toa (per anchor per blink, ~20 Hz)
    4. sync_status.csv      — /uwb/sync_status (1 Hz diagnostic)
    5. master_anchor.csv    — raw MA MASTER_CLOCK packets (~6.67 Hz)
    6. slave_anchor.csv     — raw SA MASTER + TAG packets (~30 Hz)
    7. position_compensated.csv — /state/position_compensated (bias-corrected)
    8. imu_raw.csv          — /imu/raw (IMU mentah + jangkar: ros_time/tag_ms/blink)
    9. orientation.csv      — /state/orientation (quaternion)
   10. translation_velocity.csv — /state/translation_velocity (DIFF: turunan posisi)
   11. wolf_velocity.csv    — /state/wolf_velocity (EKF: v̂ state, dipakai ZUPT)
   12. angular_velocity.csv — /state/angular_velocity (gyro terfilter dari imu_processor)
   13. translation_acceleration.csv — /state/translation_acceleration (accel terfilter)
   14. angular_acceleration.csv — /state/angular_acceleration (SG; perlu enable_angular)

master_anchor.csv and slave_anchor.csv are format-compatible with
udp_to_csv_router.py output — directly usable by self_calibrate.py
and sync.py offline tools without any modification.

Output structure:
    ~/ips_logs/
        20260504_143022/
            position.csv
            position_chan.csv
            corrected_toa.csv
            sync_status.csv
            master_anchor.csv   ← NEW: compatible with udp_to_csv_router.py
            slave_anchor.csv    ← NEW: compatible with udp_to_csv_router.py
        20260504_150112_kalibrasi/
            ...

Parameters:
    base_dir        (str)  : base directory for recordings (default: ~/ips_logs)
    auto_start_raw  (bool) : if True, start recording raw anchor data immediately
                             on node startup without waiting for service call.
                             Position/ToA layers still need explicit start call.

Service interface:
    ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'start'}"
    ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'start', label: 'test_titik_A'}"
    ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'stop'}"
    ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'status'}"
"""

import csv
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PointStamped,
    QuaternionStamped,
    Vector3Stamped,
)

from ips_msgs.msg import CorrectedToA, SyncStatus, UwbAnchorReport, ImuTelemetry
from ips_msgs.srv import RecordControl

from ips_nodes.common import (
    QOS_STATE_RELIABLE,
    QOS_SENSOR_BEST_EFFORT_DEEP,
    QOS_SENSOR_BEST_EFFORT,
    REPORT_TYPE_MASTER_CLOCK,
    REPORT_TYPE_SLAVE_MASTER,
    REPORT_TYPE_SLAVE_TAG,
)


class CsvWriter:
    """Thread-safe CSV writer that can be opened/closed per session."""

    def __init__(self):
        self._fp = None
        self._writer = None
        self._lock = threading.Lock()
        self.row_count: int = 0

    def open(self, filepath: str, header: list[str]) -> None:
        with self._lock:
            self.close_unlocked()
            self._fp = open(filepath, 'w', newline='', encoding='utf-8')
            self._writer = csv.writer(self._fp)
            self._writer.writerow(header)
            self._fp.flush()
            self.row_count = 0

    def write_row(self, row: list) -> None:
        with self._lock:
            if self._writer is None:
                return
            self._writer.writerow(row)
            self._fp.flush()
            self.row_count += 1

    def close(self) -> int:
        with self._lock:
            return self.close_unlocked()

    def close_unlocked(self) -> int:
        count = self.row_count
        if self._fp is not None:
            try:
                self._fp.close()
            except OSError:
                pass
            self._fp = None
            self._writer = None
        return count

    @property
    def is_open(self) -> bool:
        return self._writer is not None


class RecorderNode(Node):
    """On-demand CSV recorder controlled by service calls."""

    def __init__(self) -> None:
        super().__init__('recorder')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('base_dir', '~/ips_logs')
        self.declare_parameter(
            'auto_start_raw', False,
        )

        base_raw: str = self.get_parameter('base_dir').value
        self._base_dir = Path(base_raw).expanduser()
        self._auto_start_raw: bool = bool(
            self.get_parameter('auto_start_raw').value
        )

        # ---- Service ---------------------------------------------------
        self._srv = self.create_service(
            RecordControl, '~/control', self._on_control
        )

        # ---- CSV writers (one per layer) --------------------------------
        self._w_position = CsvWriter()
        self._w_chan = CsvWriter()
        self._w_toa = CsvWriter()
        self._w_sync = CsvWriter()
        # Raw anchor data — format-compatible with udp_to_csv_router.py
        self._w_master = CsvWriter()   # master_anchor.csv
        self._w_slave = CsvWriter()    # slave_anchor.csv
        self._w_position_comp = CsvWriter()  # position_compensated.csv
        # IMU layers (Tingkat 2)
        self._w_imu = CsvWriter()            # imu_raw.csv
        self._w_orient = CsvWriter()         # orientation.csv
        # Derived state layers (Tingkat 2.1) — kecepatan & percepatan
        self._w_tvel = CsvWriter()           # translation_velocity.csv (diff)
        self._w_wvel = CsvWriter()           # wolf_velocity.csv (EKF state)
        self._w_avel = CsvWriter()           # angular_velocity.csv (filtered)
        self._w_tacc = CsvWriter()           # translation_acceleration.csv
        self._w_aacc = CsvWriter()           # angular_acceleration.csv (SG)

        # ---- Recording state -------------------------------------------
        self._recording: bool = False
        self._session_dir: Optional[Path] = None
        self._start_time: Optional[float] = None
        self._state_lock = threading.Lock()

        # ---- Subscriptions (always active — write only when recording) -
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/state/position',
            self._on_position,
            QOS_STATE_RELIABLE,
        )
        self.create_subscription(
            PointStamped,
            '/state/position_chan',
            self._on_position_chan,
            QOS_STATE_RELIABLE,
        )
        self.create_subscription(
            CorrectedToA,
            '/uwb/corrected_toa',
            self._on_corrected_toa,
            QOS_STATE_RELIABLE,
        )
        self.create_subscription(
            SyncStatus,
            '/uwb/sync_status',
            self._on_sync_status,
            QOS_STATE_RELIABLE,
        )
        # Compensated position (from bias_compensator node)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/state/position_compensated',
            self._on_position_compensated,
            QOS_STATE_RELIABLE,
        )

        # Raw anchor reports — always subscribed, written when recording active
        # (or immediately if auto_start_raw=True)
        self.create_subscription(
            UwbAnchorReport,
            '/uwb/anchor_reports',
            self._on_anchor_report,
            QOS_SENSOR_BEST_EFFORT_DEEP,
        )

        # IMU layers (Tingkat 2) — BEST_EFFORT (match imu_processor/gateway)
        self.create_subscription(
            ImuTelemetry,
            '/imu/raw',
            self._on_imu_raw,
            QOS_SENSOR_BEST_EFFORT,
        )
        self.create_subscription(
            QuaternionStamped,
            '/state/orientation',
            self._on_orientation,
            QOS_STATE_RELIABLE,
        )

        # Derived state (Tingkat 2.1) — kecepatan & percepatan
        self.create_subscription(
            Vector3Stamped, '/state/translation_velocity',
            self._on_tvel, QOS_STATE_RELIABLE,
        )
        self.create_subscription(
            Vector3Stamped, '/state/wolf_velocity',
            self._on_wvel, QOS_STATE_RELIABLE,
        )
        self.create_subscription(
            Vector3Stamped, '/state/angular_velocity',
            self._on_avel, QOS_STATE_RELIABLE,
        )
        self.create_subscription(
            Vector3Stamped, '/state/translation_acceleration',
            self._on_tacc, QOS_STATE_RELIABLE,
        )
        self.create_subscription(
            Vector3Stamped, '/state/angular_acceleration',
            self._on_aacc, QOS_STATE_RELIABLE,
        )

        # ---- Auto-start raw logging if requested ----------------------
        if self._auto_start_raw:
            self._open_raw_writers(self._base_dir / 'raw_continuous')
            self.get_logger().info(
                f'auto_start_raw=True — raw anchor CSVs writing to '
                f'{self._base_dir / "raw_continuous"}'
            )

        self.get_logger().info(
            f'recorder ready — base_dir={self._base_dir}'
        )

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------
    def _on_control(
        self,
        request: RecordControl.Request,
        response: RecordControl.Response,
    ) -> RecordControl.Response:
        action = request.action.strip().lower()

        if action == 'start':
            return self._handle_start(request.label.strip(), response)
        elif action == 'stop':
            return self._handle_stop(response)
        elif action == 'status':
            return self._handle_status(response)
        else:
            response.success = False
            response.message = (
                f'unknown action "{request.action}". '
                f'Use "start", "stop", or "status".'
            )
            return response

    # ------------------------------------------------------------------
    def _handle_start(
        self, label: str, response: RecordControl.Response
    ) -> RecordControl.Response:
        with self._state_lock:
            if self._recording:
                response.success = False
                response.message = (
                    f'already recording to {self._session_dir}'
                )
                response.filepath = str(self._session_dir)
                response.duration_s = time.time() - self._start_time
                response.rows_written = self._total_rows()
                return response

            # Build session directory name
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            dirname = f'{ts}_{label}' if label else ts
            session_dir = self._base_dir / dirname
            session_dir.mkdir(parents=True, exist_ok=True)

            # Open all CSV writers
            self._w_position.open(
                str(session_dir / 'position.csv'),
                ['pc_time_s', 'ros_time_s', 'x', 'y', 'z',
                 'cov_xx', 'cov_yy', 'cov_zz'],
            )
            self._w_chan.open(
                str(session_dir / 'position_chan.csv'),
                ['pc_time_s', 'ros_time_s', 'x', 'y', 'z'],
            )
            self._w_toa.open(
                str(session_dir / 'corrected_toa.csv'),
                ['pc_time_s', 'ros_time_s',
                 'tag_id', 'slave_id', 'tag_seq', 'sync_seq',
                 'toa_corrected_s', 'toa_li_s', 'toa_kf_s',
                 'prop_delay_s', 'delta_k', 'theta_k_s', 'kf_used'],
            )
            self._w_sync.open(
                str(session_dir / 'sync_status.csv'),
                ['pc_time_s', 'ros_time_s', 'reset_count',
                 'dropped_packets_on_reset',
                 'slave_ids', 'sync_count', 'li_ready',
                 'kf_converged', 'kf_updates',
                 'delta_k', 'theta_k_s', 'kf_drift', 'kf_drift_rate'],
            )
            self._w_position_comp.open(
                str(session_dir / 'position_compensated.csv'),
                ['pc_time_s', 'ros_time_s', 'x', 'y', 'z',
                 'cov_xx', 'cov_yy', 'cov_zz'],
            )
            # IMU mentah (Tingkat 2): SEMUA jangkar sinkron disimpan —
            #   ros_time_s (jam laptop), tag_time_ms (jam tag), blink (jangkar
            #   UWB untuk alignment IMU<->posisi). seq utk deteksi paket hilang.
            self._w_imu.open(
                str(session_dir / 'imu_raw.csv'),
                ['pc_time_s', 'ros_time_s', 'seq', 'tag_time_ms', 'blink',
                 'yaw_deg', 'pitch_deg', 'roll_deg',
                 'quat_w', 'quat_x', 'quat_y', 'quat_z',
                 'gyro_x', 'gyro_y', 'gyro_z',
                 'accel_x', 'accel_y', 'accel_z'],
            )
            self._w_orient.open(
                str(session_dir / 'orientation.csv'),
                ['pc_time_s', 'ros_time_s', 'quat_w', 'quat_x', 'quat_y', 'quat_z'],
            )
            # Derived state (Tingkat 2.1) — header seragam vec3
            _vec3_hdr = ['pc_time_s', 'ros_time_s', 'x', 'y', 'z']
            self._w_tvel.open(str(session_dir / 'translation_velocity.csv'), _vec3_hdr)
            self._w_wvel.open(str(session_dir / 'wolf_velocity.csv'), _vec3_hdr)
            self._w_avel.open(str(session_dir / 'angular_velocity.csv'), _vec3_hdr)
            self._w_tacc.open(str(session_dir / 'translation_acceleration.csv'), _vec3_hdr)
            self._w_aacc.open(str(session_dir / 'angular_acceleration.csv'), _vec3_hdr)
            # Raw anchor data — same format as udp_to_csv_router.py output
            self._open_raw_writers(session_dir)

            self._session_dir = session_dir
            self._start_time = time.time()
            self._recording = True

        self.get_logger().info(f'recording STARTED → {session_dir}')
        response.success = True
        response.message = f'recording started'
        response.filepath = str(session_dir)
        response.duration_s = 0.0
        response.rows_written = 0
        return response

    def _handle_stop(
        self, response: RecordControl.Response
    ) -> RecordControl.Response:
        with self._state_lock:
            if not self._recording:
                response.success = False
                response.message = 'not recording'
                return response

            self._recording = False
            duration = time.time() - self._start_time
            total = self._total_rows()
            dirpath = str(self._session_dir)

            # Close all writers
            self._w_position.close()
            self._w_chan.close()
            self._w_toa.close()
            self._w_sync.close()
            self._w_master.close()
            self._w_slave.close()
            self._w_position_comp.close()
            self._w_imu.close()
            self._w_orient.close()
            self._w_tvel.close()
            self._w_wvel.close()
            self._w_avel.close()
            self._w_tacc.close()
            self._w_aacc.close()

        self.get_logger().info(
            f'recording STOPPED — {dirpath} '
            f'({duration:.1f}s, {total} rows total)'
        )
        response.success = True
        response.message = f'recording stopped'
        response.filepath = dirpath
        response.duration_s = duration
        response.rows_written = total
        return response

    def _handle_status(
        self, response: RecordControl.Response
    ) -> RecordControl.Response:
        with self._state_lock:
            response.success = True
            if self._recording:
                response.message = 'recording'
                response.filepath = str(self._session_dir)
                response.duration_s = time.time() - self._start_time
                response.rows_written = self._total_rows()
            else:
                response.message = 'idle'
                response.filepath = ''
                response.duration_s = 0.0
                response.rows_written = 0
        return response

    def _total_rows(self) -> int:
        return (
            self._w_position.row_count
            + self._w_chan.row_count
            + self._w_toa.row_count
            + self._w_sync.row_count
            + self._w_master.row_count
            + self._w_slave.row_count
            + self._w_position_comp.row_count
            + self._w_imu.row_count
            + self._w_orient.row_count
            + self._w_tvel.row_count
            + self._w_wvel.row_count
            + self._w_avel.row_count
            + self._w_tacc.row_count
            + self._w_aacc.row_count
        )

    # ------------------------------------------------------------------
    # Topic callbacks (write only when recording)
    # ------------------------------------------------------------------
    def _on_position(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._recording:
            return
        p = msg.pose.pose.position
        c = msg.pose.covariance
        self._w_position.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{c[0]:.6e}', f'{c[7]:.6e}', f'{c[14]:.6e}',
        ])

    def _on_position_chan(self, msg: PointStamped) -> None:
        if not self._recording:
            return
        self._w_chan.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            f'{msg.point.x:.6f}', f'{msg.point.y:.6f}', f'{msg.point.z:.6f}',
        ])

    def _on_position_compensated(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._recording:
            return
        p = msg.pose.pose.position
        c = msg.pose.covariance
        self._w_position_comp.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{c[0]:.6e}', f'{c[7]:.6e}', f'{c[14]:.6e}',
        ])

    def _on_corrected_toa(self, msg: CorrectedToA) -> None:
        if not self._recording:
            return
        self._w_toa.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            msg.tag_id, msg.slave_id, msg.tag_seq, msg.sync_seq,
            f'{msg.toa_corrected_s:.15e}',
            f'{msg.toa_li_s:.15e}',
            f'{msg.toa_kf_s:.15e}',
            f'{msg.prop_delay_s:.15e}',
            f'{msg.delta_k:.15f}',
            f'{msg.theta_k_s:.15e}',
            int(msg.kf_used),
        ])

    def _on_sync_status(self, msg: SyncStatus) -> None:
        if not self._recording:
            return
        self._w_sync.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            msg.reset_count,
            msg.dropped_packets_on_reset,
            ';'.join(str(x) for x in msg.slave_ids),
            ';'.join(str(x) for x in msg.sync_count),
            ';'.join(str(x) for x in msg.li_ready),
            ';'.join(str(x) for x in msg.kf_converged),
            ';'.join(str(x) for x in msg.kf_updates),
            ';'.join(f'{x:.15f}' for x in msg.delta_k),
            ';'.join(f'{x:.15e}' for x in msg.theta_k_s),
            ';'.join(f'{x:.15f}' for x in msg.kf_drift),
            ';'.join(f'{x:.12e}' for x in msg.kf_drift_rate),
        ])

    # ------------------------------------------------------------------
    # IMU callbacks (Tingkat 2) — write only when recording
    # ------------------------------------------------------------------
    def _on_imu_raw(self, msg: ImuTelemetry) -> None:
        if not self._recording:
            return
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self._w_imu.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            msg.seq, msg.tag_time_ms, msg.blink,
            f'{msg.yaw_deg:.4f}', f'{msg.pitch_deg:.4f}', f'{msg.roll_deg:.4f}',
            f'{msg.quat_w:.6f}', f'{msg.quat_x:.6f}',
            f'{msg.quat_y:.6f}', f'{msg.quat_z:.6f}',
            f'{g.x:.6f}', f'{g.y:.6f}', f'{g.z:.6f}',
            f'{a.x:.6f}', f'{a.y:.6f}', f'{a.z:.6f}',
        ])

    def _on_orientation(self, msg: QuaternionStamped) -> None:
        if not self._recording:
            return
        q = msg.quaternion
        self._w_orient.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            f'{q.w:.6f}', f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}',
        ])

    # ---- Derived state (Tingkat 2.1) ----
    def _write_vec3(self, writer, msg) -> None:
        if not self._recording:
            return
        v = msg.vector
        writer.write_row([
            f'{time.time():.6f}',
            self._ros_time_s(msg.header.stamp),
            f'{v.x:.6f}', f'{v.y:.6f}', f'{v.z:.6f}',
        ])

    def _on_tvel(self, msg: Vector3Stamped) -> None:
        self._write_vec3(self._w_tvel, msg)

    def _on_wvel(self, msg: Vector3Stamped) -> None:
        self._write_vec3(self._w_wvel, msg)

    def _on_avel(self, msg: Vector3Stamped) -> None:
        self._write_vec3(self._w_avel, msg)

    def _on_tacc(self, msg: Vector3Stamped) -> None:
        self._write_vec3(self._w_tacc, msg)

    def _on_aacc(self, msg: Vector3Stamped) -> None:
        self._write_vec3(self._w_aacc, msg)

    # ------------------------------------------------------------------
    @staticmethod
    def _ros_time_s(stamp) -> str:
        return f'{float(stamp.sec) + float(stamp.nanosec) * 1e-9:.9f}'

    # ------------------------------------------------------------------
    # Raw anchor helpers
    # ------------------------------------------------------------------
    def _open_raw_writers(self, session_dir: Path) -> None:
        """Open master_anchor.csv and slave_anchor.csv in session_dir.

        Format is identical to udp_to_csv_router.py so offline tools
        (self_calibrate.py, sync.py) can consume them directly.
        """
        session_dir.mkdir(parents=True, exist_ok=True)
        # Header matches udp_to_csv_router.py CsvLogger header exactly.
        self._w_master.open(
            str(session_dir / 'master_anchor.csv'),
            ['pc_time_s', 'src_ip', 'line'],
        )
        self._w_slave.open(
            str(session_dir / 'slave_anchor.csv'),
            ['pc_time_s', 'src_ip', 'line'],
        )

    def _on_anchor_report(self, msg: UwbAnchorReport) -> None:
        """Reconstruct the original CSV string from the ROS message and log it.

        MASTER_CLOCK packets  → master_anchor.csv
        SLAVE_MASTER + SLAVE_TAG packets → slave_anchor.csv

        The reconstructed 'line' field is byte-for-byte identical to what
        udp_to_csv_router.py would have written, so self_calibrate.py and
        sync.py can parse these files without any modification.
        """
        if not self._recording:
            return

        pc = f'{msg.pc_time_s:.6f}'
        src = msg.src_ip if msg.src_ip else '0.0.0.0'

        if msg.report_type == REPORT_TYPE_MASTER_CLOCK:
            # "MASTER_CLOCK,<ma_id>,<session_id>,<seq>,<tx_hex>"
            line = (
                f'MASTER_CLOCK,{msg.reporter_id},{msg.session_id},'
                f'{msg.seq},{msg.tx_hex}'
            )
            self._w_master.write_row([pc, src, line])

        elif msg.report_type == REPORT_TYPE_SLAVE_MASTER:
            # "<sa_id>,MASTER,<ma_id>,<seq>,<tk_hex>,<rx_hex>"
            line = (
                f'{msg.reporter_id},MASTER,{msg.source_id},'
                f'{msg.seq},{msg.tx_hex},{msg.rx_hex}'
            )
            self._w_slave.write_row([pc, src, line])

        elif msg.report_type == REPORT_TYPE_SLAVE_TAG:
            # "<sa_id>,TAG,<tag_id>,<seq>,NA,<rx_hex>"
            line = (
                f'{msg.reporter_id},TAG,{msg.source_id},'
                f'{msg.seq},NA,{msg.rx_hex}'
            )
            self._w_slave.write_row([pc, src, line])

    def destroy_node(self) -> bool:
        with self._state_lock:
            if self._recording:
                self._recording = False
                self._w_position.close()
                self._w_chan.close()
                self._w_toa.close()
                self._w_sync.close()
                self._w_master.close()
                self._w_slave.close()
                self._w_position_comp.close()
                self.get_logger().info(
                    f'recording force-stopped on shutdown → {self._session_dir}'
                )
            elif self._auto_start_raw:
                # Close continuous raw writers even if no session was started
                self._w_master.close()
                self._w_slave.close()
                self._w_position_comp.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
