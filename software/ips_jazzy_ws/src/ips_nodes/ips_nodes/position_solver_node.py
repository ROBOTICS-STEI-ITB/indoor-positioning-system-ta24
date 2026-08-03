"""position_solver_node — TDoA Chan + Robust KF (real-time).

v7: StudentTFilter replaces standard KF (selectable via kf_type param).
    Fixed hybrid CCP field names.

Inputs:
    /uwb/corrected_toa  (ips_msgs/CorrectedToA, RELIABLE)
    /uwb/anchor_config  (geometry_msgs/PoseArray, TRANSIENT_LOCAL)
    /uwb/anchor_reports (ips_msgs/UwbAnchorReport, BEST_EFFORT) — for Hybrid CCP

Outputs:
    /state/position_chan  (geometry_msgs/PointStamped, RELIABLE)  — raw Chan
    /state/position       (geometry_msgs/PoseWithCovarianceStamped, RELIABLE) — KF
"""

from collections import OrderedDict
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseArray,
    PointStamped,
    PoseWithCovarianceStamped,
)

from ips_msgs.msg import CorrectedToA, UwbAnchorReport

from ips_nodes.algorithms import position as algo
from ips_nodes.common import (
    QOS_STATE_RELIABLE,
    QOS_SENSOR_BEST_EFFORT_DEEP,
    QOS_LATCHED_CONFIG,
    RX_ANCHORS_ORDERED,
    REPORT_TYPE_MASTER_CLOCK,
    REPORT_TYPE_SLAVE_MASTER,
)


class PositionSolverNode(Node):
    """Per-blink aggregator + Chan/KF solver + Hybrid Calibrator."""

    def __init__(self) -> None:
        super().__init__('position_solver')

        # ==========================================================
        # 1. PARAMETER DECLARATIONS
        # ==========================================================
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('use_kalman_filter', True)
        self.declare_parameter('blink_buffer_size', 64)
        self.declare_parameter('blink_publish_timeout_s', 0.5)

        # Antenna bias calibration
        self.declare_parameter('bias_anchor_ids', [2, 3, 4, 5])
        self.declare_parameter('bias_values_ns', [0.0, 0.0, 0.0, 0.0])

        # Chan / KF tuning
        self.declare_parameter('sigma_a', 0.05)
        self.declare_parameter('sigma_t_tdoa', 2.13e-10)
        self.declare_parameter('pos_nominal', [1.75, 1.35, 1.5])
        self.declare_parameter('room_dim', [3.4374, 2.74, 3.5])
        self.declare_parameter('dt_nominal_s', 0.20)
        self.declare_parameter('rx_anchors_ordered', list(RX_ANCHORS_ORDERED))

        # KF type selection: "standard" or "student_t"
        self.declare_parameter('kf_type', 'student_t')
        self.declare_parameter('stf_eta0', 5.0)
        self.declare_parameter('stf_alpha_max', 3.0)

        # DZS outlier filter + G1
        self.declare_parameter('dzs_enabled', True)
        self.declare_parameter('dzs_window', 100)
        self.declare_parameter('dzs_k_pos', 2.5)
        self.declare_parameter('dzs_k_neg', 3.5)
        self.declare_parameter('dzs_mad_min_m', 0.03)
        self.declare_parameter('dzs_warmup', 20)
        self.declare_parameter('dzs_log_every_s', 5.0)
        self.declare_parameter('g1_enabled', True)
        self.declare_parameter('g1_margin', 1.5)

        # Hybrid Calibration Parameters
        self.declare_parameter('hybrid_calib_enabled', True)
        self.declare_parameter('hybrid_gt_pos', [1.755, 1.377, 0.34])
        self.declare_parameter('mc_pos', [2.15, 1.345, 2.25])
        self.declare_parameter('hybrid_samples', 100)
        self.declare_parameter('ccp_calib_period_s', 30.0)

        # ==========================================================
        # 2. FETCH PARAMETER VALUES
        # ==========================================================
        self._frame_id: str = self.get_parameter('frame_id').value
        self._use_kf: bool = bool(self.get_parameter('use_kalman_filter').value)
        self._buf_size: int = int(self.get_parameter('blink_buffer_size').value)
        self._timeout_s: float = float(self.get_parameter('blink_publish_timeout_s').value)

        bias_ids = list(self.get_parameter('bias_anchor_ids').value)
        bias_ns_vals = list(self.get_parameter('bias_values_ns').value)
        self._sigma_a: float = float(self.get_parameter('sigma_a').value)
        self._sigma_t: float = float(self.get_parameter('sigma_t_tdoa').value)
        pos_nom = np.asarray(list(self.get_parameter('pos_nominal').value), dtype=float)
        room = np.asarray(list(self.get_parameter('room_dim').value), dtype=float)
        self._dt_nom: float = float(self.get_parameter('dt_nominal_s').value)

        rx_ids = [int(x) for x in self.get_parameter('rx_anchors_ordered').value]
        if len(rx_ids) != 4:
            raise RuntimeError(f'rx_anchors_ordered must have 4 entries, got {rx_ids}')
        self._rx_ids: tuple[int, ...] = tuple(rx_ids)

        # KF type
        self._kf_type: str = self.get_parameter('kf_type').value
        self._stf_eta0: float = float(self.get_parameter('stf_eta0').value)
        self._stf_alpha_max: float = float(self.get_parameter('stf_alpha_max').value)

        # Static Bias Dict
        if len(bias_ids) != len(bias_ns_vals):
            raise RuntimeError('bias_anchor_ids and bias_values_ns must have equal length')
        self._bias_s: dict[int, float] = {
            int(a): float(v) * 1e-9 for a, v in zip(bias_ids, bias_ns_vals)
        }
        for a in self._rx_ids:
            self._bias_s.setdefault(int(a), 0.0)

        # DZS State
        self._dzs_enabled: bool = bool(self.get_parameter('dzs_enabled').value)
        self._dzs_log_every_s: float = float(self.get_parameter('dzs_log_every_s').value)
        self._dzs_filter: Optional[algo.AsymmetricDZSFilter] = None
        self._dzs_k_pos = float(self.get_parameter('dzs_k_pos').value)
        self._dzs_k_neg = float(self.get_parameter('dzs_k_neg').value)
        self._dzs_window = int(self.get_parameter('dzs_window').value)
        self._dzs_mad_min_m = float(self.get_parameter('dzs_mad_min_m').value)
        self._dzs_warmup = int(self.get_parameter('dzs_warmup').value)
        self._g1_enabled = bool(self.get_parameter('g1_enabled').value)
        self._g1_margin = float(self.get_parameter('g1_margin').value)
        self._dzs_last_log_t: float = 0.0

        # Hybrid Calibration State
        self._hybrid_enabled = bool(self.get_parameter('hybrid_calib_enabled').value)
        self._hybrid_gt_pos = np.asarray(list(self.get_parameter('hybrid_gt_pos').value), dtype=float)
        self._mc_pos = np.asarray(list(self.get_parameter('mc_pos').value), dtype=float)
        self._hybrid_samples = int(self.get_parameter('hybrid_samples').value)
        self._calibrator = None
        self._is_calibrated = not self._hybrid_enabled
        self._gt_buffer = []
        self._bias_ccp_s = {int(a): 0.0 for a in self._rx_ids}
        self._bias_gt_res_s = {int(a): 0.0 for a in self._rx_ids}

        # Geometry placeholders
        self._anchor_positions: Optional[dict[int, np.ndarray]] = None
        self._A: Optional[np.ndarray] = None
        self._K: Optional[np.ndarray] = None
        self._G: Optional[np.ndarray] = None
        self._room = room
        self._pos_nom = pos_nom

        # KF (instantiated in _on_anchor_config)
        self._kf = None
        self._last_pos_chan: np.ndarray = pos_nom.copy()
        self._last_t_ref: Optional[float] = None

        # Aggregation buffer
        self._buf: "OrderedDict[int, dict[int, float]]" = OrderedDict()
        self._buf_t: dict[int, float] = {}

        # ==========================================================
        # 3. ROS SUBSCRIPTIONS & PUBLISHERS
        # ==========================================================
        self._sub_toa = self.create_subscription(
            CorrectedToA, '/uwb/corrected_toa',
            self._on_toa, QOS_STATE_RELIABLE,
        )
        self._sub_cfg = self.create_subscription(
            PoseArray, '/uwb/anchor_config',
            self._on_anchor_config, QOS_LATCHED_CONFIG,
        )
        # CCP tracking for Hybrid Calibration
        self._sub_reports = self.create_subscription(
            UwbAnchorReport, '/uwb/anchor_reports',
            self._on_anchor_report, QOS_SENSOR_BEST_EFFORT_DEEP,
        )

        self._pub_chan = self.create_publisher(
            PointStamped, '/state/position_chan', QOS_STATE_RELIABLE,
        )
        self._pub_kf = self.create_publisher(
            PoseWithCovarianceStamped, '/state/position', QOS_STATE_RELIABLE,
        )

        # Background CCP Bias update timer
        ccp_period = float(self.get_parameter('ccp_calib_period_s').value)
        self._ccp_timer = self.create_timer(ccp_period, self._update_ccp_bias)

        self.get_logger().info(
            f'position_solver ready (rx_ids={self._rx_ids}, '
            f'KF={self._use_kf}, kf_type={self._kf_type}, '
            f'DZS={self._dzs_enabled}, HybridCalib={self._hybrid_enabled})'
        )

    # ------------------------------------------------------------------
    # Anchor config handler
    # ------------------------------------------------------------------
    def _on_anchor_config(self, msg: PoseArray) -> None:
        try:
            ids = [int(x) for x in msg.header.frame_id.split(',') if x]
        except ValueError:
            self.get_logger().error('anchor_config: bad frame_id encoding')
            return
        if len(ids) != len(msg.poses):
            self.get_logger().error('anchor_config: id/pose length mismatch')
            return
        positions = {
            aid: np.array([p.position.x, p.position.y, p.position.z], dtype=float)
            for aid, p in zip(ids, msg.poses)
        }
        missing = [a for a in self._rx_ids if a not in positions]
        if missing:
            self.get_logger().error(f'anchor_config missing RX anchors: {missing}')
            return
        self._anchor_positions = positions
        ordered_xyz = [positions[a] for a in self._rx_ids]
        self._A, self._K, self._G = algo.prepare_geometry(ordered_xyz)
        rcond_G = 1.0 / np.linalg.cond(self._G)
        self.get_logger().info(
            f'anchor positions loaded for ids={self._rx_ids}, rcond(G)={rcond_G:.3e}'
        )

        # Init DZS
        if self._dzs_enabled and self._dzs_filter is None:
            self._dzs_filter = algo.AsymmetricDZSFilter(
                anchor_ids=self._rx_ids,
                reference_id=self._rx_ids[0],
                anchor_positions=self._anchor_positions,
                window=self._dzs_window,
                k_pos=self._dzs_k_pos,
                k_neg=self._dzs_k_neg,
                mad_min=self._dzs_mad_min_m,
                warmup=self._dzs_warmup,
                g1_enabled=self._g1_enabled,
                g1_margin=self._g1_margin,
            )

        # Init Calibrator
        if self._hybrid_enabled and self._calibrator is None:
            self._calibrator = algo.RealtimeHybridCalibrator(
                self._mc_pos, self._anchor_positions, self._rx_ids[0]
            )

        # Init KF — select type based on kf_type parameter
        if self._use_kf:
            R_kf = algo.compute_R_kf(self._A, self._pos_nom, self._sigma_t)
            if self._kf_type == 'student_t':
                self._kf = algo.StudentTFilter(
                    R_kf, self._pos_nom, self._sigma_a,
                    eta0=self._stf_eta0,
                    alpha_max=self._stf_alpha_max,
                )
                self.get_logger().info(
                    f'StudentTFilter created: eta0={self._stf_eta0}, '
                    f'alpha_max={self._stf_alpha_max}'
                )
            else:
                self._kf = algo.PositionKalmanFilter(
                    R_kf, self._pos_nom, self._sigma_a
                )
                self.get_logger().info('Standard PositionKalmanFilter created')

    # ------------------------------------------------------------------
    # Hybrid Calibration Background Tasks
    # ------------------------------------------------------------------
    def _on_anchor_report(self, msg: UwbAnchorReport) -> None:
        """Feed CCP packets to hybrid calibrator.

        FIXED: use correct field names from UwbAnchorReport.msg:
          report_type (not type), reporter_id (not anchor_id),
          tx_hex/rx_hex (not timestamp_dtu).
        """
        if self._calibrator is None:
            return
        if msg.report_type == REPORT_TYPE_MASTER_CLOCK:
            # MA TX timestamp — parse hex to int DTU
            if msg.tx_hex:
                try:
                    ts_dtu = int(msg.tx_hex, 16)
                    self._calibrator.push_master_tx(int(msg.seq), ts_dtu)
                except ValueError:
                    pass
        elif msg.report_type == REPORT_TYPE_SLAVE_MASTER:
            # SA RX of CCP — parse rx_hex to int DTU
            if msg.rx_hex:
                try:
                    ts_dtu = int(msg.rx_hex, 16)
                    self._calibrator.push_slave_rx(
                        int(msg.reporter_id), int(msg.seq), ts_dtu
                    )
                except ValueError:
                    pass

    def _update_ccp_bias(self) -> None:
        if self._calibrator is None:
            return
        new_bias = self._calibrator.compute_bias()
        if new_bias:
            self._bias_ccp_s.update(new_bias)
            bias_str = ', '.join(
                f'SA{k}:{v*1e9:+.2f}ns'
                for k, v in sorted(self._bias_ccp_s.items())
                if k != self._rx_ids[0]
            )
            self.get_logger().info(f'CCP Bias updated: {bias_str}')

    # ------------------------------------------------------------------
    # ToA callback (one per anchor per blink)
    # ------------------------------------------------------------------
    def _on_toa(self, msg: CorrectedToA) -> None:
        if self._A is None:
            return

        sid = int(msg.slave_id)
        if sid not in self._rx_ids:
            return
        seq = int(msg.tag_seq)

        # Total Bias: static + CCP + GT residual
        bias_total = (
            self._bias_s.get(sid, 0.0)
            + self._bias_ccp_s.get(sid, 0.0)
            + self._bias_gt_res_s.get(sid, 0.0)
        )
        toa_s = float(msg.toa_corrected_s) - bias_total

        bucket = self._buf.get(seq)
        if bucket is None:
            bucket = {}
            self._buf[seq] = bucket
            self._buf_t[seq] = self.get_clock().now().nanoseconds * 1e-9
            while len(self._buf) > self._buf_size:
                old_seq, _ = self._buf.popitem(last=False)
                self._buf_t.pop(old_seq, None)

        bucket[sid] = toa_s

        if len(bucket) == 4:
            self._solve_and_publish(seq, msg.header.stamp, bucket)

            # Non-blocking GT calibration phase
            if not self._is_calibrated:
                self._gt_buffer.append(dict(bucket))
                if len(self._gt_buffer) % 10 == 0:
                    self.get_logger().info(
                        f'GT calibration: {len(self._gt_buffer)}/{self._hybrid_samples}'
                    )
                if len(self._gt_buffer) >= self._hybrid_samples:
                    self._finish_gt_calibration()

            self._buf.pop(seq, None)
            self._buf_t.pop(seq, None)

        # Prune stale
        now = self.get_clock().now().nanoseconds * 1e-9
        for s in list(self._buf_t.keys()):
            if now - self._buf_t[s] > self._timeout_s:
                self._buf.pop(s, None)
                self._buf_t.pop(s, None)

    def _finish_gt_calibration(self) -> None:
        """Compute GT residual bias + empirical R, then switch to calibrated mode."""
        self._bias_gt_res_s = algo.compute_hybrid_residual(
            self._gt_buffer, self._anchor_positions,
            self._rx_ids[0], self._hybrid_gt_pos,
        )
        # Compute empirical R from Chan errors vs GT
        errors = []
        for b in self._gt_buffer:
            toa_ordered = [b[a] for a in self._rx_ids]
            pos_c, valid = algo.chan_estimate(
                toa_ordered, self._A, self._K, self._G,
                self._room, self._pos_nom,
            )
            if valid:
                errors.append(pos_c - self._hybrid_gt_pos)
        if len(errors) > 20 and self._kf is not None:
            R_new = algo.compute_R_from_data(
                np.array(errors), self._kf.R, min_samples=20
            )
            self._kf.R = R_new
            self.get_logger().info('R matrix updated empirically from GT data')
        res_str = ', '.join(
            f'SA{k}:{v*1e9:+.2f}ns'
            for k, v in sorted(self._bias_gt_res_s.items())
            if k != self._rx_ids[0]
        )
        self.get_logger().info(
            f'Hybrid Calibration complete: GT residual={res_str}'
        )
        self._is_calibrated = True
        self._gt_buffer.clear()

    # ------------------------------------------------------------------
    def _solve_and_publish(
        self, seq: int, stamp, bucket: dict[int, float]
    ) -> None:
        toa_ordered = [bucket[a] for a in self._rx_ids]

        # DZS outlier check
        if self._dzs_filter is not None:
            ref_toa = bucket[self._rx_ids[0]]
            tdoa_m = {
                a: algo.C_MS * (bucket[a] - ref_toa)
                for a in self._rx_ids[1:]
            }
            is_inlier, z_scores, reason = self._dzs_filter.check_and_update(tdoa_m)
            self._maybe_log_dzs_stats()
            if not is_inlier:
                self.get_logger().debug(
                    f'DZS reject seq={seq} reason={reason} z={z_scores}'
                )
                return

        pos_chan, valid = algo.chan_estimate(
            toa_ordered, self._A, self._K, self._G,
            self._room, self._last_pos_chan,
        )

        # Publish Chan raw
        chan_msg = PointStamped()
        chan_msg.header.stamp = stamp
        chan_msg.header.frame_id = self._frame_id
        chan_msg.point.x = float(pos_chan[0])
        chan_msg.point.y = float(pos_chan[1])
        chan_msg.point.z = float(pos_chan[2])
        self._pub_chan.publish(chan_msg)

        if not valid:
            self.get_logger().debug(f'Chan invalid for seq={seq}')

        z_in = pos_chan if valid else self._last_pos_chan
        if valid:
            self._last_pos_chan = pos_chan.copy()

        # dt from blink interval
        t_ref = bucket[self._rx_ids[0]]
        if self._last_t_ref is None or t_ref <= self._last_t_ref:
            dt = self._dt_nom
        else:
            dt = t_ref - self._last_t_ref
            if dt > 5.0:
                dt = self._dt_nom
        self._last_t_ref = t_ref

        # KF step (works for both PositionKalmanFilter and StudentTFilter)
        if self._use_kf and self._kf is not None:
            pos_kf = self._kf.step(z_in, dt)
        else:
            pos_kf = z_in

        # Publish KF position
        kf_msg = PoseWithCovarianceStamped()
        kf_msg.header.stamp = stamp
        kf_msg.header.frame_id = self._frame_id
        kf_msg.pose.pose.position.x = float(pos_kf[0])
        kf_msg.pose.pose.position.y = float(pos_kf[1])
        kf_msg.pose.pose.position.z = float(pos_kf[2])
        kf_msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        if self._use_kf and self._kf is not None:
            P = self._kf.P[:3, :3]
            for i in range(3):
                for j in range(3):
                    cov[i * 6 + j] = float(P[i, j])
        kf_msg.pose.covariance = cov
        self._pub_kf.publish(kf_msg)

    # ------------------------------------------------------------------
    def _maybe_log_dzs_stats(self) -> None:
        if self._dzs_filter is None or self._dzs_log_every_s <= 0.0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._dzs_last_log_t) < self._dzs_log_every_s:
            return
        self._dzs_last_log_t = now
        s = self._dzs_filter.stats()
        n = s['n_total']
        if n == 0:
            return
        rej_pct = 100.0 * s['reject_rate']
        thr = self._dzs_filter.threshold_cm()
        parts = []
        for sid in sorted(thr.keys()):
            v = thr[sid]
            if v is None:
                parts.append(f'SA{sid}=N/A')
            else:
                parts.append(f'SA{sid}=+{v["pos"]:.1f}/-{v["neg"]:.1f}cm')
        thr_str = ', '.join(parts)
        g1_str = (
            f' g1_reject={s["n_g1_reject"]}'
            if s.get('n_g1_reject', 0) > 0 else ''
        )
        self.get_logger().info(
            f'DZS stats: n={n} inliers={s["n_inliers"]} '
            f'outliers={s["n_outliers"]} ({rej_pct:.1f}%){g1_str} '
            f'thresholds [{thr_str}]'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PositionSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
