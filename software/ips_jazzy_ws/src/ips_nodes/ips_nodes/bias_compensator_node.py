"""bias_compensator_node — Static bias compensator for IPS positioning.

Sits between position_solver and downstream consumers (state_aggregator,
differentiator). Subscribes to raw position, applies a calibrated 3D
offset, and publishes compensated position.

State machine:
    IDLE         → no bias known; passthrough (bias = 0)
    CALIBRATING  → collecting samples at known GT position
    OPERATIONAL  → bias known; subtracting from all output

Calibration is triggered via the ~/calibrate service:
    ros2 service call /bias_compensator/calibrate ips_msgs/srv/Calibrate \\
        "{gt_x: 1.5, gt_y: 1.0, gt_z: 1.15, n_samples: 300, skip_warmup: 30}"

Bias is auto-saved to YAML if bias_yaml_path parameter is set, and
auto-loaded on next launch — so calibration persists across sessions.

Inputs:
    /state/position                (geometry_msgs/PoseWithCovarianceStamped)
Outputs:
    /state/position_compensated    (geometry_msgs/PoseWithCovarianceStamped)
Service:
    ~/calibrate                    (ips_msgs/srv/Calibrate)
"""

import threading
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from ips_msgs.srv import Calibrate

from ips_nodes.common import QOS_STATE_RELIABLE


class BiasCompensatorNode(Node):
    """Static bias compensator with calibration support."""

    STATE_IDLE        = 0
    STATE_CALIBRATING = 1
    STATE_OPERATIONAL = 2

    def __init__(self) -> None:
        super().__init__('bias_compensator')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('bias_yaml_path', '')
        self.declare_parameter('auto_load', True)
        self.declare_parameter('auto_save', True)
        self.declare_parameter('n_samples_default', 300)
        self.declare_parameter('skip_warmup_default', 30)
        self.declare_parameter('robust', True)
        self.declare_parameter('log_status_every_s', 5.0)
        self.declare_parameter('calibration_timeout_s', 60.0)

        self._yaml_path: str = self.get_parameter('bias_yaml_path').value
        self._auto_load: bool = self.get_parameter('auto_load').value
        self._auto_save: bool = self.get_parameter('auto_save').value
        self._n_default: int = int(self.get_parameter('n_samples_default').value)
        self._skip_default: int = int(self.get_parameter('skip_warmup_default').value)
        self._robust: bool = self.get_parameter('robust').value
        self._log_every: float = float(self.get_parameter('log_status_every_s').value)
        self._calib_timeout: float = float(
            self.get_parameter('calibration_timeout_s').value
        )

        # ---- State (protected by self._lock) -----------------------------
        self._lock = threading.Lock()
        self._state: int = self.STATE_IDLE
        self._bias = np.zeros(3)
        self._bias_std = np.zeros(3)

        # ---- Calibration state (protected by self._calib_lock) -----------
        self._calib_lock = threading.Lock()
        self._calib_running: bool = False
        self._calib_done_event = threading.Event()
        self._calib_n_target: int = 0
        self._calib_skip: int = 0
        self._calib_gt = np.zeros(3)
        self._calib_samples: List[np.ndarray] = []
        self._calib_warmup_count: int = 0
        self._calib_result: Optional[dict] = None

        # ---- Auto-load saved bias ----------------------------------------
        if self._auto_load and self._yaml_path:
            self._try_load_bias()

        # ---- ROS callback groups (allow service to run alongside sub) ----
        sub_cb = ReentrantCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()

        # ---- Subscription, publisher, service ----------------------------
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/state/position',
            self._on_position,
            QOS_STATE_RELIABLE,
            callback_group=sub_cb,
        )
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/state/position_compensated',
            QOS_STATE_RELIABLE,
        )
        self._srv = self.create_service(
            Calibrate, '~/calibrate', self._on_calibrate,
            callback_group=srv_cb,
        )

        if self._log_every > 0:
            self.create_timer(self._log_every, self._log_status)

        self.get_logger().info(
            f"bias_compensator ready  state={self._state_name()}  "
            f"bias=({self._bias[0]*100:+.2f}, {self._bias[1]*100:+.2f}, "
            f"{self._bias[2]*100:+.2f}) cm"
        )

    # ---------------------------------------------------------------------
    def _state_name(self) -> str:
        return ['IDLE', 'CALIBRATING', 'OPERATIONAL'][self._state]

    # ---------------------------------------------------------------------
    # Subscription: applies bias and publishes; collects samples if calibrating
    # ---------------------------------------------------------------------
    def _on_position(self, msg: PoseWithCovarianceStamped) -> None:
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

        # ---- Sample collection (if calibrating) ----
        with self._calib_lock:
            if self._calib_running:
                self._calib_warmup_count += 1
                if self._calib_warmup_count > self._calib_skip:
                    self._calib_samples.append(pos.copy())
                    n_needed = self._calib_n_target - self._calib_skip
                    if len(self._calib_samples) >= n_needed:
                        # Compute and store result (under both locks)
                        self._finalize_calibration_locked()

        # ---- Apply bias and publish ----
        with self._lock:
            bias = self._bias.copy()
        corrected = pos - bias

        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose.position.x = float(corrected[0])
        out.pose.pose.position.y = float(corrected[1])
        out.pose.pose.position.z = float(corrected[2])
        out.pose.pose.orientation = msg.pose.pose.orientation
        out.pose.covariance = msg.pose.covariance
        self._pub.publish(out)

    # ---------------------------------------------------------------------
    # Service: calibrate (blocks until done or timeout)
    # ---------------------------------------------------------------------
    def _on_calibrate(self, request, response):
        # Reject if a calibration is already running
        with self._calib_lock:
            if self._calib_running:
                response.success = False
                response.message = "Calibration already in progress"
                return response

        n_samples = request.n_samples if request.n_samples > 0 else self._n_default
        skip = request.skip_warmup if request.skip_warmup > 0 else self._skip_default
        if skip >= n_samples:
            response.success = False
            response.message = (
                f"skip_warmup ({skip}) must be < n_samples ({n_samples})"
            )
            return response

        gt = np.array([request.gt_x, request.gt_y, request.gt_z])

        # ---- Begin calibration ----
        with self._calib_lock:
            self._calib_running = True
            self._calib_done_event.clear()
            self._calib_n_target = n_samples
            self._calib_skip = skip
            self._calib_gt = gt
            self._calib_samples = []
            self._calib_warmup_count = 0
            self._calib_result = None
        with self._lock:
            self._state = self.STATE_CALIBRATING

        self.get_logger().info(
            f"Calibration started: GT=({gt[0]:.3f}, {gt[1]:.3f}, {gt[2]:.3f})  "
            f"n_samples={n_samples}  skip_warmup={skip}"
        )

        # ---- Wait for completion (or timeout) ----
        finished = self._calib_done_event.wait(timeout=self._calib_timeout)
        if not finished:
            with self._calib_lock:
                self._calib_running = False
            with self._lock:
                # Stay in IDLE if no prior bias, otherwise revert to OPERATIONAL
                self._state = (
                    self.STATE_OPERATIONAL
                    if np.any(self._bias != 0.0)
                    else self.STATE_IDLE
                )
            response.success = False
            response.message = (
                f"Calibration timeout ({self._calib_timeout}s). "
                f"Got {len(self._calib_samples)}/{n_samples - skip} samples. "
                f"Is position_solver publishing on /state/position?"
            )
            return response

        result = self._calib_result
        response.success = True
        response.message = (
            f"Calibration done with {result['n']} samples. "
            f"Bias: ({result['bias'][0]*100:+.2f}, "
            f"{result['bias'][1]*100:+.2f}, "
            f"{result['bias'][2]*100:+.2f}) cm"
        )
        response.bias_x = float(result['bias'][0])
        response.bias_y = float(result['bias'][1])
        response.bias_z = float(result['bias'][2])
        response.std_x  = float(result['std'][0])
        response.std_y  = float(result['std'][1])
        response.std_z  = float(result['std'][2])
        response.samples_used = int(result['n'])
        return response

    # ---------------------------------------------------------------------
    # Called from _on_position when enough samples have arrived.
    # Already holds self._calib_lock; acquires self._lock for state update.
    # ---------------------------------------------------------------------
    def _finalize_calibration_locked(self) -> None:
        samples = np.array(self._calib_samples)
        residuals = samples - self._calib_gt

        if self._robust:
            bias = np.median(residuals, axis=0)
            mad = np.median(np.abs(residuals - bias), axis=0)
            std = 1.4826 * mad
        else:
            bias = np.mean(residuals, axis=0)
            std = np.std(residuals, axis=0)

        with self._lock:
            self._bias = bias
            self._bias_std = std
            self._state = self.STATE_OPERATIONAL

        self._calib_result = {
            'bias': bias,
            'std':  std,
            'n':    len(samples),
            'gt':   self._calib_gt.copy(),
        }
        self._calib_running = False

        self.get_logger().info(
            f"Calibration complete:\n"
            f"  bias = ({bias[0]*100:+.2f}, {bias[1]*100:+.2f}, "
            f"{bias[2]*100:+.2f}) cm\n"
            f"  std  = ({std[0]*1000:.1f}, {std[1]*1000:.1f}, "
            f"{std[2]*1000:.1f}) mm\n"
            f"  n_samples = {len(samples)}"
        )

        if self._auto_save and self._yaml_path:
            self._try_save_bias()

        self._calib_done_event.set()

    # ---------------------------------------------------------------------
    # Persistence
    # ---------------------------------------------------------------------
    def _try_load_bias(self) -> None:
        try:
            path = Path(self._yaml_path).expanduser()
            if not path.exists():
                self.get_logger().info(
                    f"bias yaml not found at {path}, starting in IDLE mode"
                )
                return
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            b = data.get('bias', {})
            self._bias = np.array([
                float(b.get('x', 0.0)),
                float(b.get('y', 0.0)),
                float(b.get('z', 0.0)),
            ])
            self._bias_std = np.array([
                float(b.get('std_x', 0.0)),
                float(b.get('std_y', 0.0)),
                float(b.get('std_z', 0.0)),
            ])
            if np.any(self._bias != 0.0):
                self._state = self.STATE_OPERATIONAL
            self.get_logger().info(
                f"Loaded bias from {path}: ({self._bias[0]*100:+.2f}, "
                f"{self._bias[1]*100:+.2f}, {self._bias[2]*100:+.2f}) cm"
            )
        except Exception as exc:
            self.get_logger().warning(f"load_bias failed: {exc}")

    def _try_save_bias(self) -> None:
        try:
            path = Path(self._yaml_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                'bias': {
                    'x':     float(self._bias[0]),
                    'y':     float(self._bias[1]),
                    'z':     float(self._bias[2]),
                    'std_x': float(self._bias_std[0]),
                    'std_y': float(self._bias_std[1]),
                    'std_z': float(self._bias_std[2]),
                },
                'calibration_gt': {
                    'x': float(self._calib_gt[0]),
                    'y': float(self._calib_gt[1]),
                    'z': float(self._calib_gt[2]),
                },
                'n_samples': int(len(self._calib_samples)),
                'timestamp': datetime.now().isoformat(timespec='seconds'),
                'robust':    bool(self._robust),
            }
            with open(path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"Saved bias to {path}")
        except Exception as exc:
            self.get_logger().warning(f"save_bias failed: {exc}")

    # ---------------------------------------------------------------------
    def _log_status(self) -> None:
        with self._lock:
            state = self._state
            bias = self._bias.copy()
        with self._calib_lock:
            calib_n = len(self._calib_samples) if self._calib_running else 0
            calib_target = (
                self._calib_n_target - self._calib_skip
                if self._calib_running else 0
            )

        if state == self.STATE_CALIBRATING:
            self.get_logger().info(
                f"[CALIBRATING] {calib_n}/{calib_target} samples collected"
            )
        elif state == self.STATE_OPERATIONAL:
            self.get_logger().info(
                f"[OPERATIONAL] bias=({bias[0]*100:+.2f}, "
                f"{bias[1]*100:+.2f}, {bias[2]*100:+.2f}) cm"
            )
        # IDLE: stay quiet


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BiasCompensatorNode()
    # Multi-threaded so service callback can wait while sub callback runs
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
