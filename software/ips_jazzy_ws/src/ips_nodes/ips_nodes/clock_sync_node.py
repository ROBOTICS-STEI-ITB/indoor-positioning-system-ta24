"""clock_sync_node — LI-KF clock synchronization (Zhang et al., 2024).

Subscribes to raw UWB anchor reports + session events, drives an
internal SyncEngine, and emits a CorrectedToA message for every blink
that completes synchronization (Step 2 LI + Step 3 KF + Step 4 prop delay).

Inputs:
    /uwb/anchor_reports  (ips_msgs/UwbAnchorReport, BEST_EFFORT)
    /uwb/session_events  (ips_msgs/SessionEvent,    RELIABLE)

Outputs:
    /uwb/corrected_toa   (ips_msgs/CorrectedToA, RELIABLE)
    /uwb/sync_status     (ips_msgs/SyncStatus,   RELIABLE)
"""

import threading

import rclpy
from rclpy.node import Node

from ips_msgs.msg import (
    UwbAnchorReport,
    SessionEvent,
    CorrectedToA,
    SyncStatus,
)

from ips_nodes.algorithms import sync_engine as se
from ips_nodes.common import (
    QOS_SENSOR_BEST_EFFORT_DEEP,
    QOS_STATE_RELIABLE,
    REPORT_TYPE_MASTER_CLOCK,
    REPORT_TYPE_SLAVE_MASTER,
    REPORT_TYPE_SLAVE_TAG,
    EVENT_RESET,
    EVENT_AUTO_RESTART,
    DTU_S,
)


class ClockSyncNode(Node):
    """Wraps SyncEngine in a ROS2 node with online callback emission."""

    def __init__(self) -> None:
        super().__init__('clock_sync')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('status_period_s', 1.0)
        self.declare_parameter('kf_enabled', True)
        # Optional CSV dump for offline analysis (default off in ROS2).
        self.declare_parameter('write_debug_csv', False)

        self._status_period: float = float(self.get_parameter('status_period_s').value)
        kf_enabled: bool = bool(self.get_parameter('kf_enabled').value)
        write_csv: bool = bool(self.get_parameter('write_debug_csv').value)

        # Apply KF flag to the algorithm module before instantiating engine.
        # se.KF_ENABLED = kf_enabled

        # ---- Engine ----------------------------------------------------
        # Lock guards engine + emission queue from concurrent access
        # (subscription callbacks are invoked from rclpy executor threads).
        self._engine_lock = threading.Lock()
        self._engine = se.SyncEngine(
            toa_callback=self._on_corrected_toa_record,
            write_csv=write_csv,
            kf_enabled=kf_enabled, # <--  BARIS Update 7 May
        )

        # ---- Subscriptions --------------------------------------------
        self._sub_reports = self.create_subscription(
            UwbAnchorReport,
            '/uwb/anchor_reports',
            self._on_report,
            QOS_SENSOR_BEST_EFFORT_DEEP,
        )
        self._sub_events = self.create_subscription(
            SessionEvent,
            '/uwb/session_events',
            self._on_event,
            QOS_STATE_RELIABLE,
        )

        # ---- Publishers -----------------------------------------------
        self._pub_toa = self.create_publisher(
            CorrectedToA, '/uwb/corrected_toa', QOS_STATE_RELIABLE
        )
        self._pub_status = self.create_publisher(
            SyncStatus, '/uwb/sync_status', QOS_STATE_RELIABLE
        )

        # ---- Status timer ---------------------------------------------
        self._status_timer = self.create_timer(
            self._status_period, self._publish_status
        )

        self.get_logger().info(
            f'clock_sync ready (KF={kf_enabled}, write_csv={write_csv})'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_report(self, msg: UwbAnchorReport) -> None:
        try:
            with self._engine_lock:
                if msg.report_type == REPORT_TYPE_MASTER_CLOCK:
                    self._engine.add_master_clock(
                        msg.reporter_id,
                        msg.session_id,
                        msg.seq,
                        msg.tx_hex,
                    )
                elif msg.report_type == REPORT_TYPE_SLAVE_MASTER:
                    self._engine.add_slave_master_rx(
                        msg.reporter_id,
                        msg.source_id,
                        msg.seq,
                        msg.tx_hex,
                        msg.rx_hex,
                    )
                elif msg.report_type == REPORT_TYPE_SLAVE_TAG:
                    self._engine.add_slave_tag_rx(
                        msg.reporter_id,
                        msg.source_id,
                        msg.seq,
                        msg.rx_hex,
                    )
        except Exception as exc:
            self.get_logger().warning(f'engine ingest error: {exc}')

    def _on_event(self, msg: SessionEvent) -> None:
        if msg.kind in (EVENT_RESET, EVENT_AUTO_RESTART):
            with self._engine_lock:
                self._engine.handle_ma_reset(msg.device_id, msg.session_id)
            self.get_logger().warning(
                f'engine reset triggered by event kind={msg.kind} '
                f'device={msg.device_id} session={msg.session_id}'
            )

    # ------------------------------------------------------------------
    # Engine callback (runs INSIDE _on_report under engine_lock)
    # ------------------------------------------------------------------
    def _on_corrected_toa_record(self, rec: dict) -> None:
        """Called by SyncEngine for every (slave, blink) pair that completes."""
        msg = CorrectedToA()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uwb'
        msg.tag_id = int(rec['tag_id'])
        msg.slave_id = int(rec['slave_id'])
        msg.tag_seq = int(rec['tag_seq'])
        msg.sync_seq = int(rec['sync_seq'])
        msg.toa_corrected_s = float(rec['toa_final_dtu']) * DTU_S
        msg.toa_li_s = float(rec['toa_li_dtu']) * DTU_S
        msg.toa_kf_s = float(rec['toa_kf_dtu']) * DTU_S
        msg.prop_delay_s = float(rec['prop_delay_dtu']) * DTU_S
        msg.delta_k = float(rec['delta_k'])
        msg.theta_k_s = float(rec['theta_k_dtu']) * DTU_S
        msg.kf_used = bool(rec['kf_used'])
        self._pub_toa.publish(msg)

    # ------------------------------------------------------------------
    # Status snapshot
    # ------------------------------------------------------------------
    def _publish_status(self) -> None:
        with self._engine_lock:
            status = self._engine.get_sync_status()
            reset_count = self._engine.reset_count
            dropped = self._engine.dropped_packets_on_reset

        msg = SyncStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uwb'
        msg.reset_count = reset_count
        msg.dropped_packets_on_reset = dropped

        ids = sorted(status.keys())
        msg.slave_ids = ids
        msg.sync_count = [int(status[i]['sync_count']) for i in ids]
        msg.li_ready = [bool(status[i]['li_ready']) for i in ids]
        msg.kf_converged = [bool(status[i]['kf_converged']) for i in ids]
        msg.kf_updates = [int(status[i]['kf_updates']) for i in ids]
        msg.delta_k = [float(status[i]['delta_k'] or 0.0) for i in ids]
        # theta_k_us in engine status → convert to seconds for the message
        msg.theta_k_s = [
            float((status[i]['theta_k_us'] or 0.0) * 1e-6) for i in ids
        ]
        msg.kf_drift = [float(status[i]['kf_drift'] or 0.0) for i in ids]
        msg.kf_drift_rate = [
            float(status[i]['kf_drift_rate'] or 0.0) for i in ids
        ]
        self._pub_status.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        try:
            with self._engine_lock:
                self._engine.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClockSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
