"""
=============================================================================
 LI-KF Clock Synchronization Engine for UWB TDoA Positioning
=============================================================================

 Unified implementation combining:
   - Production-ready DW1000 infrastructure (DTU integer arithmetic,
     40-bit wrap handling, seq-based matching, CSV audit logging)
   - 3-State Kalman Filter (Zhang et al., 2024) for nonlinear clock
     drift and frequency drift elimination

 Algorithm (Zhang et al., IEEE IoT Journal, 2024):
   Step 1: Linear Interpolation    - Compute δ_k (clock drift) and θ_k (offset)
   Step 2: Raw ToA Correction      - Convert SA timestamps to MA domain
   Step 3: 3-State Kalman Filter   - Refine drift with [ToA, δ, ϕ] state
   Step 4: Propagation Delay Comp. - Subtract known MA→SA physical delay

 Hardware: DW1000 (Decawave/Qorvo)
 Topology: 1 Master Anchor + N Slave Anchors + M Tags

=============================================================================
"""

import csv
import math
import time
import threading
import re
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Tuple
from pathlib import Path

# =============================================================================
# CONFIGURATION
# =============================================================================

# --- File I/O ---
MASTER_CSV_FILE = "master_anchor.csv"
SLAVE_CSV_FILE  = "slave_anchor.csv"

STEP1_OUTPUT_CSV = "sync_step1_results.csv"
STEP2_OUTPUT_CSV = "sync_step2_results.csv"
KF_OUTPUT_CSV    = "sync_kf_state.csv"

# --- DW1000 Constants ---
DW1000_CLOCK_HZ = 499.2e6 * 128.0          # 63,897,600,000 Hz
DTU_SECONDS     = 1.0 / DW1000_CLOCK_HZ    # ~15.65 ps per tick
WRAP_40         = 1 << 40
HALF_WRAP_40    = 1 << 39

# --- Propagation Delay Compensation (Step 4) ---
# In TDoA mode, propagation delay cancels in cross-anchor differencing,
# so this is normally left empty. Set only if you need absolute ToA.
# Format: {slave_id: delay_in_microseconds}
PROPAGATION_DELAY_US = {}

# --- Kalman Filter Parameters (Step 3) ---
# Measurement noise variance (DW1000 spec: σ² ≈ 1.5e-20 s²)
# Converted to DTU²: (1.5e-20 s²) / (DTU_SECONDS²)
KF_MEAS_NOISE_R_S2 = 1.5e-20   # in seconds²

# Process noise covariance Q (diagonal, tunable)
# These control the trade-off between responsiveness and smoothing.
# Larger Q → faster tracking of changes but more noise pass-through.
# Smaller Q → smoother but slower response.
KF_PROC_NOISE_Q_TOA    = 5.0e-20   # seconds²
KF_PROC_NOISE_Q_DRIFT  = 1.0e-24   # (dimensionless)²
KF_PROC_NOISE_Q_FDRIFT = 1.0e-28   # (1/s)²

# Initial state covariance P0 (diagonal)
KF_INIT_P_TOA    = 1.0e-18
KF_INIT_P_DRIFT  = 1.0e-20
KF_INIT_P_FDRIFT = 1.0e-24

# Minimum sync cycles before KF output is trusted
KF_MIN_CONVERGENCE = 5

# Enable/disable Step 3 (Kalman Filter)
KF_ENABLED = True

# =============================================================================
# REGEX PARSERS (from original code)
# =============================================================================

MASTER_HUMAN_RE = re.compile(
    r"ANCHOR\s+CLOCK\s+TX\s*\|\s*SEQ=(\d+)\s*\|\s*CLOCK=([0-9A-Fa-f]+)"
)
# Master CCP with session_id: MASTER_CLOCK,<ma>,<session>,<seq>,<hex>
MASTER_CSV_RE = re.compile(
    r"MASTER_CLOCK\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*([0-9A-Fa-f]+)"
)

# SA tag reception: <sa>,TAG,<tag>,<seq>,NA,<hex>
SLAVE_TAG_RE = re.compile(
    r"(\d+)\s*,\s*TAG\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*NA\s*,\s*([0-9A-Fa-f]+)"
)
# SA CCP reception: <sa>,MASTER,<ma>,<seq>,<tk_hex>,<rx_hex>
SLAVE_MASTER_RE = re.compile(
    r"(\d+)\s*,\s*MASTER\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*([0-9A-Fa-f]+)\s*,\s*([0-9A-Fa-f]+)"
)

# Control packets
RESET_RE = re.compile(r"^RESET\s*,\s*MA\s*,\s*(\d+)\s*,\s*(\d+)\s*,")
HELLO_RE = re.compile(r"^HELLO\s*,\s*MA\s*,\s*(\d+)\s*,\s*(\d+)\s*,")

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def hex40_to_int(hex_str: str) -> int:
    return int(hex_str, 16)

def dtu_to_seconds(x) -> float:
    return float(x) * DTU_SECONDS

def dtu_to_nanoseconds(x) -> float:
    return dtu_to_seconds(x) * 1e9

def dtu_to_microseconds(x) -> float:
    return dtu_to_seconds(x) * 1e6

def seconds_to_dtu(s: float) -> int:
    return int(round(s / DTU_SECONDS))

def microseconds_to_dtu(us: float) -> int:
    return int(round((us * 1e-6) / DTU_SECONDS))

def fmt_sec(x: float) -> str:
    return f"{x:.12e}"

# =============================================================================
# 40-BIT TIMESTAMP UNWRAPPER
# =============================================================================

class TimestampUnwrapper40:
    """Handles DW1000 40-bit counter wrap-around (~17.2s period)."""

    def __init__(self, name: str = ""):
        self.name = name
        self.last_raw: Optional[int] = None
        self.wrap_count: int = 0

    def unwrap(self, raw_40: int) -> int:
        if self.last_raw is None:
            self.last_raw = raw_40
            return raw_40

        if raw_40 < self.last_raw and (self.last_raw - raw_40) > HALF_WRAP_40:
            self.wrap_count += 1
        elif raw_40 > self.last_raw and (raw_40 - self.last_raw) > HALF_WRAP_40:
            pass  # backward anomaly, ignore

        self.last_raw = raw_40
        return raw_40 + self.wrap_count * WRAP_40

    def reset(self):
        self.last_raw = None
        self.wrap_count = 0

# =============================================================================
# 3x3 MATRIX & 3x1 VECTOR (minimal, no numpy dependency)
# =============================================================================

def mat3_zero() -> List[List[float]]:
    return [[0.0]*3 for _ in range(3)]

def mat3_identity() -> List[List[float]]:
    m = mat3_zero()
    m[0][0] = m[1][1] = m[2][2] = 1.0
    return m

def mat3_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    C = mat3_zero()
    for i in range(3):
        for j in range(3):
            for k in range(3):
                C[i][j] += A[i][k] * B[k][j]
    return C

def mat3_add(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    return [[A[i][j] + B[i][j] for j in range(3)] for i in range(3)]

def mat3_transpose(A: List[List[float]]) -> List[List[float]]:
    return [[A[j][i] for j in range(3)] for i in range(3)]

def mat3_vec_mul(A: List[List[float]], x: List[float]) -> List[float]:
    return [sum(A[i][j] * x[j] for j in range(3)) for i in range(3)]

# =============================================================================
# 3-STATE KALMAN FILTER — CORRECTED IMPLEMENTATION
# =============================================================================
#
# Zhang's KF tracks the master-domain ToA as a state that advances in time.
#
# State X = [ToA, rate, accel]
#   X[0] = last synchronized ToA in master domain (= Tk at CCP instants)
#   X[1] = rate of advancement (≈ 1.0 seconds/second for perfect clock)
#   X[2] = rate of change of rate (≈ 0, captures slow frequency drift)
#
# State transition (constant-acceleration model):
#   F = | 1    dt    dt²/2 |
#       | 0    1     dt    |
#       | 0    0     1     |
#
# Observation at each CCP: z = Tk (exact master time, from LI Step 2 at CCP)
# H = [1, 0, 0]
#
# For blinks between CCPs: use KF PREDICTION (not LI correction)
#   predicted_toa = X[0] + X[1]*dt + 0.5*X[2]*dt²
#   where dt = time since last KF update, in master-domain seconds
#
# WHY THIS WORKS:
# - At CCPs: observation z=Tk is exact → KF corrects any prediction error
# - X[1] converges to 1.0 (master time advances at 1 second per second)
# - Between CCPs: prediction extrapolates linearly → unbiased
# - LI correction has 2d*tau bias, but KF prediction doesn't
# =============================================================================

class KalmanFilter3State:
    """
    Three-state Kalman filter for clock synchronization.
    Predicts master-domain ToA using constant-acceleration model.
    """

    def __init__(self):
        self.X: List[float] = [0.0, 1.0, 0.0]  # [ToA, rate≈1.0, accel≈0]

        self.P: List[List[float]] = [
            [KF_INIT_P_TOA,    0.0,              0.0],
            [0.0,              KF_INIT_P_DRIFT,   0.0],
            [0.0,              0.0,              KF_INIT_P_FDRIFT],
        ]

        self.Q: List[List[float]] = [
            [KF_PROC_NOISE_Q_TOA,    0.0,                   0.0],
            [0.0,                    KF_PROC_NOISE_Q_DRIFT,  0.0],
            [0.0,                    0.0,                   KF_PROC_NOISE_Q_FDRIFT],
        ]

        self.R: float = KF_MEAS_NOISE_R_S2

        self.initialized: bool = False
        self.update_count: int = 0
        self.last_update_Tk_s: float = 0.0  # master time at last KF update

    def initialize(self, Tk_s: float):
        """Initialize with first CCP master timestamp."""
        self.X = [Tk_s, 1.0, 0.0]  # rate = 1.0 (NOT delta_k!)
        self.last_update_Tk_s = Tk_s
        self.initialized = True
        self.update_count = 0

    def predict_and_update(self, dt_s: float, z_Tk_s: float):
        """
        Predict then update with new CCP observation.
        dt_s: time since last CCP in seconds (master domain)
        z_Tk_s: master timestamp Tk at this CCP (observation)
        """
        if not self.initialized:
            return

        # --- Predict ---
        dt = dt_s
        dt2 = dt * dt

        # F * X
        X_pred = [
            self.X[0] + self.X[1] * dt + 0.5 * self.X[2] * dt2,
            self.X[1] + self.X[2] * dt,
            self.X[2],
        ]

        # F * P * F^T + Q  (manually expanded for 3x3)
        P = self.P
        # FP = F @ P
        FP = [
            [P[0][0] + dt*P[1][0] + 0.5*dt2*P[2][0],
             P[0][1] + dt*P[1][1] + 0.5*dt2*P[2][1],
             P[0][2] + dt*P[1][2] + 0.5*dt2*P[2][2]],
            [P[1][0] + dt*P[2][0],
             P[1][1] + dt*P[2][1],
             P[1][2] + dt*P[2][2]],
            [P[2][0], P[2][1], P[2][2]],
        ]
        # P_pred = FP @ F^T + Q
        P_pred = [[0.0]*3 for _ in range(3)]
        Ft_col = [[1, 0, 0], [dt, 1, 0], [0.5*dt2, dt, 1]]  # F^T columns
        for i in range(3):
            for j in range(3):
                val = 0.0
                for k in range(3):
                    val += FP[i][k] * Ft_col[j][k]
                P_pred[i][j] = val + self.Q[i][j]

        # --- Update ---
        # H = [1, 0, 0]
        y = z_Tk_s - X_pred[0]
        S = P_pred[0][0] + self.R

        if abs(S) < 1e-50:
            return

        K = [P_pred[i][0] / S for i in range(3)]

        self.X = [X_pred[i] + K[i] * y for i in range(3)]

        # P = (I - K*H) * P_pred
        P_new = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                P_new[i][j] = P_pred[i][j] - K[i] * P_pred[0][j]
        self.P = P_new

        self.last_update_Tk_s = z_Tk_s
        self.update_count += 1

    def predict_toa(self, dt_master_s: float) -> float:
        """
        Predict the corrected ToA at dt_master_s after last KF update.
        This is the KEY output function for blink correction.
        
        dt_master_s: time since last CCP, in MASTER-domain seconds
                     Approximate with: LI_corrected_blink - Tk
                     or: Delta / delta_k (rough, but good enough for dt)
        
        Returns: predicted ToA in master domain (seconds)
        """
        dt = dt_master_s
        return self.X[0] + self.X[1] * dt + 0.5 * self.X[2] * dt * dt

    @property
    def converged(self) -> bool:
        return self.initialized and self.update_count >= KF_MIN_CONVERGENCE

    @property
    def drift(self) -> float:
        """Rate of ToA advancement (should be ≈1.0)."""
        return self.X[1]

    @property
    def drift_rate(self) -> float:
        """Acceleration of ToA advancement."""
        return self.X[2]

# =============================================================================
# CSV RESULT WRITER
# =============================================================================

class CsvResultWriter:
    def __init__(self, filepath: str, header: list, enabled: bool = True):
        self.filepath = filepath
        self.lock = threading.Lock()
        self.enabled = enabled
        if not enabled:
            self.fp = None
            self.writer = None
            return
        self.fp = open(filepath, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fp)
        self.writer.writerow(header)
        self.fp.flush()

    def write_row(self, row):
        if not self.enabled:
            return
        with self.lock:
            self.writer.writerow(row)
            self.fp.flush()

    def close(self):
        if not self.enabled:
            return
        with self.lock:
            try:
                self.fp.close()
            except Exception:
                pass

# =============================================================================
# PER-SLAVE SYNCHRONIZATION CONTEXT
# =============================================================================

class SlaveAnchorContext:
    """Holds all synchronization state for one slave anchor."""

    def __init__(self, slave_id: int):
        self.slave_id = slave_id

        # Single unwrapper shared for both R_k (CCP) and ToA (tag blink),
        # because both come from the SAME physical clock (SA's timer).
        # Using separate unwrappers would cause wrap-count desync.
        self.clock_unwrapper = TimestampUnwrapper40(f"slave{slave_id}_clock")

        # CCP pair state
        self.prev_sync: Optional[dict] = None
        self.curr_sync: Optional[dict] = None

        # Step 1 results
        self.delta_k: Optional[float] = None  # clock drift ratio
        self.theta_k: Optional[int] = None     # clock offset (DTU)
        self.li_ready: bool = False

        # Step 3: Kalman Filter
        self.kf = KalmanFilter3State()
        self.last_ccp_interval_s: float = 0.150  # will be computed from data

        # Step 4: Propagation delay
        self.prop_delay_dtu: int = 0
        if slave_id in PROPAGATION_DELAY_US:
            self.prop_delay_dtu = microseconds_to_dtu(PROPAGATION_DELAY_US[slave_id])

        # Statistics
        self.sync_count: int = 0
        self.last_sync_error_ns: float = 0.0

    def reset(self):
        self.prev_sync = None
        self.curr_sync = None
        self.delta_k = None
        self.theta_k = None
        self.li_ready = False
        self.kf = KalmanFilter3State()
        self.sync_count = 0
        self.clock_unwrapper.reset()

# =============================================================================
# MAIN SYNC ENGINE
# =============================================================================

class SyncEngine:
    """
    LI-KF Synchronization Engine.
    
    Manages clock synchronization for multiple slave anchors relative
    to a single master anchor, using the LI-KF algorithm.
    
    Data flow:
      1. Master CSV → add_master_clock()   → stores T_k by seq
      2. Slave CSV  → add_slave_master_rx() → stores R_k by seq, triggers sync
      3. Slave CSV  → add_slave_tag_rx()    → queues tag blink, triggers correction
    
    Internal processing:
      - When both T_k and R_k arrive for same seq → _try_build_sync_pair()
      - When new sync pair completes → _run_step1() then _run_step3_kf()
      - When tag blink arrives → _process_pending_tags() → Step2 + KF correction
    """

    def __init__(self, toa_callback=None, write_csv: bool = True, kf_enabled: bool = True):
        """
        Args:
            toa_callback: Optional callback fn(record: dict) called for every
                corrected ToA. The record contains all step2 fields. Used by
                ROS2 wrapper to emit CorrectedToA messages in real time.
                If None, results are only written to CSV.
            write_csv: If False, skip step1/step2/kf CSV writing (useful for
                ROS2 mode where the CSV would just be debug bloat).
            kf_enabled: Enable or disable Step 3 Kalman Filter.
        """
        self.lock = threading.Lock()

        # ROS2 integration hook
        self._toa_callback = toa_callback
        self._write_csv = write_csv
        self._kf_enabled = kf_enabled # <-- SIMPAN SEBAGAI INSTANCE VARIABLE (update 7 May)

        # Master data indexed by seq
        self.master_clock_by_seq: Dict[int, dict] = {}

        # Slave data indexed by (slave_id, seq) for multi-slave support
        self.slave_sync_by_seq: Dict[Tuple[int, int], dict] = {}

        # Per-slave context
        self.slave_contexts: Dict[int, SlaveAnchorContext] = {}

        # Pending tag reports - PER SLAVE to avoid head-of-line blocking
        self.pending_tags_by_slave: Dict[int, deque] = {}

        # Master timestamp unwrapper
        self.master_tk_unwrapper = TimestampUnwrapper40("master_tk")

        # --- MA session tracking (for restart detection) ---
        self.current_ma_session: Optional[int] = None
        self.current_ma_seq_max: int = -1       # highest MA seq seen in current session
        self.reset_count: int = 0
        self.dropped_packets_on_reset: int = 0

        # Threshold: if new MA seq < last_seq - SEQ_RESET_THRESHOLD → restart detected
        # Even without session_id (old firmware backward-compat)
        self.SEQ_RESET_THRESHOLD = 100

        # CSV writers (disabled when running in ROS2 mode)
        self.step1_writer = CsvResultWriter(STEP1_OUTPUT_CSV, [
            "slave_id",
            "sync_seq_prev", "sync_seq_curr",
            "Tk_prev_hex", "Tk_prev_raw40_dtu", "Tk_prev_unwrapped_dtu", "Tk_prev_us",
            "Rk_prev_hex", "Rk_prev_raw40_dtu", "Rk_prev_unwrapped_dtu", "Rk_prev_us",
            "Tk_hex", "Tk_raw40_dtu", "Tk_unwrapped_dtu", "Tk_us",
            "Rk_hex", "Rk_raw40_dtu", "Rk_unwrapped_dtu", "Rk_us",
            "delta_k", "theta_k_dtu", "theta_k_us",
        ], enabled=self._write_csv)

        self.step2_writer = CsvResultWriter(STEP2_OUTPUT_CSV, [
            "tag_seq", "slave_id", "tag_id", "sync_used",
            # Raw ToA
            "raw_toa_hex", "raw_toa_raw40_dtu", "raw_toa_unwrapped_dtu", "raw_toa_us",
            # Sync parameters
            "Tk_dtu", "Tk_us", "Rk_dtu", "Rk_us",
            "delta_k", "theta_k_dtu", "theta_k_us",
            # Step 2: LI correction
            "Delta_dtu", "Delta_us",
            "li_corrected_toa_dtu", "li_corrected_toa_us",
            # Step 3: KF correction
            "kf_enabled",
            "kf_drift", "kf_drift_rate",
            "kf_corrected_toa_dtu", "kf_corrected_toa_us",
            # Step 4: Propagation delay
            "prop_delay_dtu", "prop_delay_us",
            "final_corrected_toa_dtu", "final_corrected_toa_us",
        ], enabled=self._write_csv)

        self.kf_writer = CsvResultWriter(KF_OUTPUT_CSV, [
            "slave_id", "sync_seq",
            "kf_state_toa_s", "kf_state_drift", "kf_state_drift_rate",
            "kf_P00", "kf_P11", "kf_P22",
            "kf_update_count", "kf_converged",
        ], enabled=self._write_csv)

    # -----------------------------------------------------------------
    # Slave context management
    # -----------------------------------------------------------------
    def _get_or_create_slave(self, slave_id: int) -> SlaveAnchorContext:
        if slave_id not in self.slave_contexts:
            self.slave_contexts[slave_id] = SlaveAnchorContext(slave_id)
        return self.slave_contexts[slave_id]

    # -----------------------------------------------------------------
    # Reset handling
    # -----------------------------------------------------------------
    def _reset_all_state(self, reason: str):
        """Clear all synchronization state. Called on MA restart."""
        self.reset_count += 1

        # Count pending tags being dropped
        pending_count = sum(len(q) for q in self.pending_tags_by_slave.values())
        self.dropped_packets_on_reset += pending_count

        print(f"\n{'=' * 72}")
        print(f"  🔴 RESET #{self.reset_count}: {reason}")
        print(f"  Clearing: {len(self.master_clock_by_seq)} master CCPs, "
              f"{len(self.slave_sync_by_seq)} sync pairs, "
              f"{pending_count} pending tags")
        print(f"{'=' * 72}\n")

        # Clear everything
        self.master_clock_by_seq.clear()
        self.slave_sync_by_seq.clear()
        self.pending_tags_by_slave.clear()

        # Reset unwrappers
        self.master_tk_unwrapper.reset()
        for ctx in self.slave_contexts.values():
            ctx.clock_unwrapper.reset()
            # Also reset per-slave sync context
            ctx.li_ready = False
            ctx.curr_sync = None
            ctx.prev_sync = None
            ctx.delta_k = 1.0
            ctx.theta_k = 0
            # Reset KF
            ctx.kf.update_count = 0
            ctx.kf.initialized = False

        self.current_ma_seq_max = -1

    def handle_ma_reset(self, master_id: int, session_id: int):
        """Called when RESET packet received from MA."""
        with self.lock:
            if self.current_ma_session is None:
                # First RESET seen — just record session
                self.current_ma_session = session_id
                print(f"[INFO] MA{master_id} initial session: {session_id}")
            elif self.current_ma_session != session_id:
                self._reset_all_state(f"MA{master_id} session change "
                                       f"{self.current_ma_session} → {session_id}")
                self.current_ma_session = session_id

    def handle_ma_hello(self, master_id: int, session_id: int):
        """Called when HELLO packet received."""
        with self.lock:
            if self.current_ma_session is None:
                self.current_ma_session = session_id
            elif self.current_ma_session != session_id:
                # Session changed without RESET — still a restart
                self._reset_all_state(f"MA{master_id} silent session change "
                                       f"{self.current_ma_session} → {session_id}")
                self.current_ma_session = session_id

    # -----------------------------------------------------------------
    # Input handlers
    # -----------------------------------------------------------------
    def add_master_clock(self, master_id: int, session_id: int, seq: int, tk_hex: str):
        # --- Session change detection ---
        if session_id != 0:  # new firmware with session_id
            if self.current_ma_session is None:
                self.current_ma_session = session_id
                print(f"[INFO] MA{master_id} detected session: {session_id}")
            elif self.current_ma_session != session_id:
                with self.lock:
                    self._reset_all_state(
                        f"MA{master_id} session change via CCP "
                        f"{self.current_ma_session} → {session_id}")
                    self.current_ma_session = session_id

        # --- Seq reset detection (backward-compat for old firmware) ---
        if self.current_ma_seq_max > 0 and seq < self.current_ma_seq_max - self.SEQ_RESET_THRESHOLD:
            with self.lock:
                self._reset_all_state(
                    f"MA seq reset detected: {self.current_ma_seq_max} → {seq}")
        self.current_ma_seq_max = max(self.current_ma_seq_max, seq)

        tk_raw40 = hex40_to_int(tk_hex)
        tk_unwrapped = self.master_tk_unwrapper.unwrap(tk_raw40)

        with self.lock:
            self.master_clock_by_seq[seq] = {
                "master_id": master_id,
                "seq": seq,
                "Tk": tk_unwrapped,
                "Tk_raw40": tk_raw40,
                "Tk_hex": tk_hex.upper(),
            }
            # Try to build sync pair for all known slaves at this seq
            for sid in list(self.slave_contexts.keys()):
                self._try_build_sync_pair(sid, seq)

    def add_slave_master_rx(self, slave_id: int, source_id: int, seq: int,
                            remote_clock_hex: str, rx_hex: str):
        ctx = self._get_or_create_slave(slave_id)
        rk_raw40 = hex40_to_int(rx_hex)
        rk_unwrapped = ctx.clock_unwrapper.unwrap(rk_raw40)

        with self.lock:
            self.slave_sync_by_seq[(slave_id, seq)] = {
                "slave_id": slave_id,
                "source_id": source_id,
                "seq": seq,
                "remote_clock_hex": remote_clock_hex.upper(),
                "Rk": rk_unwrapped,
                "Rk_raw40": rk_raw40,
                "Rk_hex": rx_hex.upper(),
            }
            self._try_build_sync_pair(slave_id, seq)

    def add_slave_tag_rx(self, slave_id: int, tag_id: int,
                         seq: int, toa_hex: str):
        ctx = self._get_or_create_slave(slave_id)
        toa_raw40 = hex40_to_int(toa_hex)
        toa_unwrapped = ctx.clock_unwrapper.unwrap(toa_raw40)

        with self.lock:
            if slave_id not in self.pending_tags_by_slave:
                self.pending_tags_by_slave[slave_id] = deque()

            self.pending_tags_by_slave[slave_id].append({
                "slave_id": slave_id,
                "tag_id": tag_id,
                "tag_seq": seq,
                "ToA": toa_unwrapped,
                "ToA_raw40": toa_raw40,
                "ToA_hex": toa_hex.upper(),
            })
            self._process_pending_tags()

    # -----------------------------------------------------------------
    # Internal: Build sync pair
    # -----------------------------------------------------------------
    def _try_build_sync_pair(self, slave_id: int, seq: int):
        if seq not in self.master_clock_by_seq:
            return
        if (slave_id, seq) not in self.slave_sync_by_seq:
            return

        master_rec = self.master_clock_by_seq[seq]
        slave_rec = self.slave_sync_by_seq[(slave_id, seq)]

        # Cross-check
        if slave_rec["remote_clock_hex"] != master_rec["Tk_hex"]:
            print(f"\n[WARNING] SA{slave_id} seq={seq}: remote_clock mismatch "
                  f"(master={master_rec['Tk_hex']}, slave_echo={slave_rec['remote_clock_hex']})")

        ctx = self._get_or_create_slave(slave_id)

        new_sync = {
            "seq": seq,
            "Tk": master_rec["Tk"],
            "Tk_raw40": master_rec["Tk_raw40"],
            "Tk_hex": master_rec["Tk_hex"],
            "Rk": slave_rec["Rk"],
            "Rk_raw40": slave_rec["Rk_raw40"],
            "Rk_hex": slave_rec["Rk_hex"],
            "master_id": master_rec["master_id"],
            "slave_id": slave_id,
        }

        if ctx.curr_sync is None:
            ctx.curr_sync = new_sync
            print(f"\n[SYNC] SA{slave_id}: First pair received (seq={seq}), waiting for next.")
            return

        if ctx.curr_sync["seq"] == seq:
            return

        if seq > ctx.curr_sync["seq"]:
            saved_prev = ctx.prev_sync
            saved_curr = ctx.curr_sync
            ctx.prev_sync = ctx.curr_sync
            ctx.curr_sync = new_sync
            step1_ok = self._run_step1(ctx)
            if not step1_ok:
                # Glitch detected: rollback curr_sync agar blink tidak pakai Tk/Rk glitch.
                # prev_sync tetap = curr_sync lama (yang baik), curr_sync dikembalikan.
                ctx.prev_sync = saved_prev
                ctx.curr_sync = saved_curr
                # Tidak panggil step3_kf atau _process_pending_tags karena state tidak berubah
                return
            self._run_step3_kf(ctx)
            self._process_pending_tags()

    # -----------------------------------------------------------------
    # Step 1: Linear Interpolation
    # -----------------------------------------------------------------
    def _run_step1(self, ctx: SlaveAnchorContext) -> bool:
        """Returns True if delta_k updated successfully, False if glitch detected."""
        if ctx.prev_sync is None or ctx.curr_sync is None:
            return False

        Tk_1 = ctx.prev_sync["Tk"]
        Rk_1 = ctx.prev_sync["Rk"]
        Tk   = ctx.curr_sync["Tk"]
        Rk   = ctx.curr_sync["Rk"]

        denom = Rk - Rk_1
        if denom == 0:
            print(f"\n[ERROR] SA{ctx.slave_id}: Step 1 failed (Rk - Rk_1 = 0)")
            ctx.li_ready = False
            return False

        new_delta_k = (Tk - Tk_1) / denom

        # === GLITCH PROTECTION ===
        # DW1000 occasionally produces glitched RX timestamps (delta_k jauh dari 1.0).
        # Crystal drift fisik typical ±50 ppm = |delta_k - 1| < 5e-5.
        # Threshold 0.001 (1000 ppm) sudah sangat longgar; apa pun di luar ini = glitch.
        DELTA_K_MAX_DEV = 0.001
        if abs(new_delta_k - 1.0) > DELTA_K_MAX_DEV:
            print(f"  [GLITCH] SA{ctx.slave_id} CCP seq={ctx.curr_sync['seq']}: "
                  f"delta_k={new_delta_k:.6f} rejected. CCP frame discarded.")
            return False  # Caller will rollback curr_sync

        ctx.delta_k = new_delta_k
        ctx.theta_k = Tk - Rk
        ctx.li_ready = True
        ctx.sync_count += 1

        # Compute actual CCP interval for KF
        ctx.last_ccp_interval_s = dtu_to_seconds(Tk - Tk_1)

        # CSV logging
        self.step1_writer.write_row([
            ctx.slave_id,
            ctx.prev_sync["seq"], ctx.curr_sync["seq"],
            ctx.prev_sync["Tk_hex"], ctx.prev_sync["Tk_raw40"], Tk_1,
            f"{dtu_to_microseconds(Tk_1):.6f}",
            ctx.prev_sync["Rk_hex"], ctx.prev_sync["Rk_raw40"], Rk_1,
            f"{dtu_to_microseconds(Rk_1):.6f}",
            ctx.curr_sync["Tk_hex"], ctx.curr_sync["Tk_raw40"], Tk,
            f"{dtu_to_microseconds(Tk):.6f}",
            ctx.curr_sync["Rk_hex"], ctx.curr_sync["Rk_raw40"], Rk,
            f"{dtu_to_microseconds(Rk):.6f}",
            f"{ctx.delta_k:.15f}", ctx.theta_k,
            f"{dtu_to_microseconds(ctx.theta_k):.6f}",
        ])

        # Console
        print(f"\n{'='*72}")
        print(f"STEP 1 - LINEAR INTERPOLATION (SA{ctx.slave_id})")
        print(f"{'-'*72}")
        print(f"  Sync k-1 seq={ctx.prev_sync['seq']}  "
              f"Tk_1={Tk_1}  Rk_1={Rk_1}")
        print(f"  Sync k   seq={ctx.curr_sync['seq']}  "
              f"Tk={Tk}  Rk={Rk}")
        print(f"  δ_k = {ctx.delta_k:.15f}")
        print(f"  θ_k = {ctx.theta_k} DTU ({dtu_to_microseconds(ctx.theta_k):.3f} μs)")
        print(f"  CCP interval = {ctx.last_ccp_interval_s*1000:.3f} ms")

        return True 

    # -----------------------------------------------------------------
    # Step 3: Kalman Filter Update
    # -----------------------------------------------------------------
    def _run_step3_kf(self, ctx: SlaveAnchorContext):
        if not self._kf_enabled:
            return
        if not ctx.li_ready or ctx.curr_sync is None:
            return

        kf = ctx.kf
        T_ccp_s = ctx.last_ccp_interval_s
        Tk_s = dtu_to_seconds(ctx.curr_sync["Tk"])

        if not kf.initialized:
            kf.initialize(Tk_s)
            print(f"  [KF] SA{ctx.slave_id}: Initialized "
                  f"(Tk={Tk_s:.12e}, rate=1.0)")
        else:
            kf.predict_and_update(T_ccp_s, Tk_s)

        # Log KF state
        self.kf_writer.write_row([
            ctx.slave_id, ctx.curr_sync["seq"],
            f"{kf.X[0]:.15e}", f"{kf.drift:.15f}", f"{kf.drift_rate:.15e}",
            f"{kf.P[0][0]:.6e}", f"{kf.P[1][1]:.6e}", f"{kf.P[2][2]:.6e}",
            kf.update_count, kf.converged,
        ])

        print(f"  [KF] SA{ctx.slave_id}: ToA={kf.X[0]:.12e} "
              f"rate={kf.drift:.12f} "
              f"accel={kf.drift_rate:.6e} "
              f"updates={kf.update_count}")

    # -----------------------------------------------------------------
    # Step 2 + 3 + 4: Tag Blink ToA Correction
    # -----------------------------------------------------------------
    def _process_pending_tags(self):
        """Process pending tag blinks. Each slave queue is independent,
        so a not-ready slave doesn't block others."""
        for slave_id in list(self.pending_tags_by_slave.keys()):
            queue = self.pending_tags_by_slave[slave_id]

            if slave_id not in self.slave_contexts:
                continue
            ctx = self.slave_contexts[slave_id]

            # Skip if this slave's sync isn't ready yet
            if not ctx.li_ready or ctx.curr_sync is None:
                continue

            Tk = ctx.curr_sync["Tk"]
            Rk = ctx.curr_sync["Rk"]
            sync_seq = ctx.curr_sync["seq"]

            while queue:
                tag = queue[0]

                # Skip tags whose ToA is before current sync
                if tag["ToA"] <= Rk:
                    print(f"\n[INFO] SA{slave_id}: Tag seq={tag['tag_seq']} "
                          f"skipped (ToA <= Rk)")
                    queue.popleft()
                    continue

                queue.popleft()

                # ---- Step 2: Linear Interpolation correction ----
                # Convert slave-domain interval to master-domain by MULTIPLYING
                # by delta_k (= master_rate/slave_rate ≈ 1/(1+d))
                Delta = tag["ToA"] - Rk
                li_corrected_toa = Tk + (Delta * ctx.delta_k)
                li_corrected_int = int(round(li_corrected_toa))

                # ---- Step 3: Kalman Filter refinement ----
                # KF PREDICTS the corrected ToA directly, avoiding LI's 2d*tau bias.
                # dt_master = approximate master-domain interval since last CCP
                #           = Delta / delta_k (rough estimate, used only for dt)
                kf_corrected_int = li_corrected_int
                kf_drift_val = ctx.delta_k
                kf_drift_rate_val = 0.0
                kf_used = False

                if self._kf_enabled and ctx.kf.converged:
                    kf_drift_val = ctx.kf.drift
                    kf_drift_rate_val = ctx.kf.drift_rate

                    # dt in master domain: convert slave interval using delta_k
                    dt_master_s = dtu_to_seconds(Delta) * ctx.delta_k
                    # KF prediction: extrapolate from last CCP
                    kf_predicted_s = ctx.kf.predict_toa(dt_master_s)
                    kf_corrected_int = seconds_to_dtu(kf_predicted_s)
                    kf_used = True

                # ---- Step 4: Propagation delay compensation ----
                # Add MA→SA propagation delay to correct for the fact that
                # SA received the CCP later than MA sent it
                final_corrected_int = kf_corrected_int + ctx.prop_delay_dtu

                # ---- CSV logging (skipped in ROS2 mode if write_csv=False) ----
                if self._write_csv:
                    self.step2_writer.write_row([
                        tag["tag_seq"], slave_id, tag["tag_id"], sync_seq,
                        tag["ToA_hex"], tag["ToA_raw40"], tag["ToA"],
                        f"{dtu_to_microseconds(tag['ToA']):.6f}",
                        Tk, f"{dtu_to_microseconds(Tk):.6f}",
                        Rk, f"{dtu_to_microseconds(Rk):.6f}",
                        f"{ctx.delta_k:.15f}", ctx.theta_k,
                        f"{dtu_to_microseconds(ctx.theta_k):.6f}",
                        Delta, f"{dtu_to_microseconds(Delta):.6f}",
                        li_corrected_int, f"{dtu_to_microseconds(li_corrected_int):.6f}",
                        kf_used,
                        f"{kf_drift_val:.15f}", f"{kf_drift_rate_val:.12e}",
                        kf_corrected_int, f"{dtu_to_microseconds(kf_corrected_int):.6f}",
                        ctx.prop_delay_dtu,
                        f"{dtu_to_microseconds(ctx.prop_delay_dtu):.6f}",
                        final_corrected_int, f"{dtu_to_microseconds(final_corrected_int):.6f}",
                    ])

                # ---- Real-time callback (ROS2 hook) ----
                if self._toa_callback is not None:
                    try:
                        self._toa_callback({
                            "tag_seq": tag["tag_seq"],
                            "tag_id": tag["tag_id"],
                            "slave_id": slave_id,
                            "sync_seq": sync_seq,
                            "toa_li_dtu": li_corrected_int,
                            "toa_kf_dtu": kf_corrected_int,
                            "toa_final_dtu": final_corrected_int,
                            "prop_delay_dtu": ctx.prop_delay_dtu,
                            "delta_k": ctx.delta_k,
                            "theta_k_dtu": ctx.theta_k,
                            "kf_used": kf_used,
                            "kf_drift": kf_drift_val,
                            "kf_drift_rate": kf_drift_rate_val,
                        })
                    except Exception as exc:
                        print(f"[WARN] toa_callback raised: {exc}")

                # ---- Console ----
                print(f"\n{'='*72}")
                print(f"STEP 2+3+4 - ToA CORRECTION (SA{slave_id}, Tag{tag['tag_id']})")
                print(f"{'-'*72}")
                print(f"  Tag seq={tag['tag_seq']}  Sync used={sync_seq}")
                print(f"  Raw ToA       : {tag['ToA']} DTU ({tag['ToA_hex']})")
                print(f"  Δ = ToA - Rk  : {Delta} DTU ({dtu_to_microseconds(Delta):.3f} μs)")
                print(f"  Step 2 (LI)   : {li_corrected_int} DTU "
                      f"({dtu_to_nanoseconds(li_corrected_int):.3f} ns)")
                if kf_used:
                    print(f"  Step 3 (KF)   : {kf_corrected_int} DTU "
                          f"({dtu_to_nanoseconds(kf_corrected_int):.3f} ns)")
                    print(f"    KF drift    : {kf_drift_val:.15f}")
                    print(f"    KF drft_rate: {kf_drift_rate_val:.6e}")
                    diff = kf_corrected_int - li_corrected_int
                    print(f"    KF-LI diff  : {diff} DTU ({dtu_to_nanoseconds(diff):.3f} ns)")
                else:
                    print(f"  Step 3 (KF)   : disabled/not converged")
                if ctx.prop_delay_dtu != 0:
                    print(f"  Step 4 (prop) : -{ctx.prop_delay_dtu} DTU "
                          f"(-{dtu_to_microseconds(ctx.prop_delay_dtu):.3f} μs)")
                print(f"  FINAL ToA     : {final_corrected_int} DTU "
                      f"({dtu_to_nanoseconds(final_corrected_int):.3f} ns)")
                print(f"{'='*72}")

    # -----------------------------------------------------------------
    # Public API for external use (future UDP/TCP integration)
    # -----------------------------------------------------------------
    def correct_tag_toa(self, slave_id: int, raw_toa_dtu: int) -> Optional[int]:
        """
        Correct a single tag blink ToA from slave's clock to master's domain.
        Returns corrected ToA in DTU, or None if sync not ready.
        
        This is the function you'd call from UDP/TCP handler.
        """
        with self.lock:
            if slave_id not in self.slave_contexts:
                return None
            ctx = self.slave_contexts[slave_id]
            if not ctx.li_ready or ctx.curr_sync is None:
                return None

            Tk = ctx.curr_sync["Tk"]
            Rk = ctx.curr_sync["Rk"]
            Delta = raw_toa_dtu - Rk

            if Delta <= 0:
                return None

            # Step 2: LI correction (MULTIPLY by delta_k)
            li_corrected = Tk + (Delta * ctx.delta_k)

            # Step 3: KF prediction (unbiased)
            if self._kf_enabled and ctx.kf.converged:
                dt_master_s = dtu_to_seconds(Delta) * ctx.delta_k
                corrected = seconds_to_dtu(ctx.kf.predict_toa(dt_master_s))
            else:
                corrected = int(round(li_corrected))

            # Step 4: Propagation delay (ADD MA→SA delay)
            corrected += ctx.prop_delay_dtu

            return corrected

    def get_sync_status(self) -> dict:
        """Get synchronization status for all slave anchors."""
        status = {}
        for sid, ctx in self.slave_contexts.items():
            status[sid] = {
                "sync_count": ctx.sync_count,
                "li_ready": ctx.li_ready,
                "delta_k": ctx.delta_k,
                "theta_k_us": dtu_to_microseconds(ctx.theta_k) if ctx.theta_k else None,
                "kf_converged": ctx.kf.converged if self._kf_enabled else False, # <-- ubah 7 may
                "kf_drift": ctx.kf.drift if ctx.kf.initialized else None,
                "kf_drift_rate": ctx.kf.drift_rate if ctx.kf.initialized else None,
                "kf_updates": ctx.kf.update_count,
            }
        return status

    def close(self):
        self.step1_writer.close()
        self.step2_writer.close()
        self.kf_writer.close()

# =============================================================================
# PARSERS (unchanged from original)
# =============================================================================

def parse_master_line(line: str):
    """Returns ('master_clock', master_id, session_id, seq, hex) or
               ('reset', master_id, session_id) or
               ('hello', master_id, session_id)"""
    m = MASTER_HUMAN_RE.search(line)
    if m:
        return ("master_clock", 1, 0, int(m.group(1)), m.group(2).upper())

    m = MASTER_CSV_RE.search(line)
    if m:
        return ("master_clock", int(m.group(1)), int(m.group(2)),
                int(m.group(3)), m.group(4).upper())

    m = RESET_RE.match(line.strip())
    if m:
        return ("reset", int(m.group(1)), int(m.group(2)))
    m = HELLO_RE.match(line.strip())
    if m:
        return ("hello", int(m.group(1)), int(m.group(2)))

    return None


def parse_slave_line(line: str):
    """Returns ('slave_master', sa, ma, seq, tk_hex, rx_hex) or
               ('slave_tag', sa, tag, seq, rx_hex)"""
    s = line.strip()

    m = SLAVE_MASTER_RE.fullmatch(s)
    if m:
        return ("slave_master", int(m.group(1)), int(m.group(2)),
                int(m.group(3)), m.group(4).upper(), m.group(5).upper())

    m = SLAVE_TAG_RE.fullmatch(s)
    if m:
        return ("slave_tag", int(m.group(1)), int(m.group(2)),
                int(m.group(3)), m.group(4).upper())

    m = RESET_RE.match(s)
    if m:
        return ("reset", int(m.group(1)), int(m.group(2)))

    return None

# =============================================================================
# CSV READER
# =============================================================================

def csv_reader(csv_file: str, parser_func, callback, label: str):
    try:
        with open(csv_file, "r", newline="", encoding="utf-8") as fp:
            reader = csv.DictReader(fp)
            prev_pc_time = None
            row_count = 0

            print(f"[INFO] Reading CSV {label}: {csv_file}")

            for row in reader:
                if "line" not in row:
                    print(f"[ERROR] {csv_file}: missing 'line' column")
                    return

                line = (row["line"] or "").strip()
                if not line:
                    continue

                if USE_REPLAY_TIMING:
                    try:
                        curr_pc_time = float(row["pc_time_s"])
                    except Exception:
                        curr_pc_time = None

                    if prev_pc_time is not None and curr_pc_time is not None:
                        dt = curr_pc_time - prev_pc_time
                        if dt > 0:
                            time.sleep(dt / max(REPLAY_SPEED, 1e-9))

                    if curr_pc_time is not None:
                        prev_pc_time = curr_pc_time

                parsed = parser_func(line)
                if parsed is not None:
                    callback(parsed)
                    row_count += 1

            print(f"[INFO] Finished {label}: {row_count} lines processed.")

    except FileNotFoundError:
        print(f"[ERROR] File not found: {csv_file}")
    except Exception as e:
        print(f"[ERROR] CSV reader {csv_file}: {e}")

# =============================================================================
# CALLBACK ROUTERS (used only when this module is run as a script)
# =============================================================================

def _make_offline_handlers(engine):
    """Build closure handlers bound to a specific engine instance."""

    def handle_master_parsed(parsed):
        typ = parsed[0]
        if typ == "master_clock":
            _, master_id, session_id, seq, clock_hex = parsed
            engine.add_master_clock(master_id, session_id, seq, clock_hex)
        elif typ == "reset":
            _, master_id, session_id = parsed
            engine.handle_ma_reset(master_id, session_id)
        elif typ == "hello":
            _, master_id, session_id = parsed
            engine.handle_ma_hello(master_id, session_id)

    def handle_slave_parsed(parsed):
        typ = parsed[0]
        if typ == "slave_master":
            _, slave_id, source_id, seq, remote_clock_hex, rx_hex = parsed
            engine.add_slave_master_rx(slave_id, source_id, seq, remote_clock_hex, rx_hex)
        elif typ == "slave_tag":
            _, slave_id, tag_id, seq, rx_hex = parsed
            engine.add_slave_tag_rx(slave_id, tag_id, seq, rx_hex)
        elif typ == "reset":
            _, master_id, session_id = parsed
            engine.handle_ma_reset(master_id, session_id)

    return handle_master_parsed, handle_slave_parsed

# =============================================================================
# MAIN
# =============================================================================

def read_csv_with_time(csv_file, parser_func, label):
    """Read CSV and return list of (pc_time, parsed_data, label)."""
    events = []
    if not Path(csv_file).exists():
        print(f"[ERROR] File not found: {csv_file}")
        return events
    with open(csv_file, "r", newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            line = (row.get("line") or "").strip()
            if not line:
                continue
            try:
                pc_time = float(row.get("pc_time_s", 0))
            except Exception:
                pc_time = 0
            parsed = parser_func(line)
            if parsed is not None:
                events.append((pc_time, parsed, label))
    print(f"[INFO] Loaded {len(events)} events from {label} ({csv_file})")
    return events


def main():
    print("=" * 72)
    print("  LI-KF Clock Synchronization Engine (single-thread merge)")
    print("  Zhang et al. (IEEE IoT Journal, 2024)")
    print("=" * 72)
    print(f"  Master CSV : {MASTER_CSV_FILE}")
    print(f"  Slave CSV  : {SLAVE_CSV_FILE}")
    print(f"  KF Enabled : {KF_ENABLED}")
    print(f"  Prop Delay : {PROPAGATION_DELAY_US}")
    print(f"  Output     : {STEP1_OUTPUT_CSV}, {STEP2_OUTPUT_CSV}, {KF_OUTPUT_CSV}")
    print("=" * 72)
    print()

    engine = SyncEngine(write_csv=True)
    handle_master_parsed, handle_slave_parsed = _make_offline_handlers(engine)

    # --- Load both CSVs ---
    master_events = read_csv_with_time(MASTER_CSV_FILE, parse_master_line, "MASTER")
    slave_events  = read_csv_with_time(SLAVE_CSV_FILE,  parse_slave_line,  "SLAVE")

    # --- Merge-sort by pc_time (chronological order) ---
    # Tie-breaker: master events before slave events at same timestamp
    # (so session change is detected before slave data for that moment)
    def sort_key(ev):
        pc_time, parsed, label = ev
        # master(0) comes before slave(1) on ties to ensure reset is processed first
        return (pc_time, 0 if label == "MASTER" else 1)

    all_events = sorted(master_events + slave_events, key=sort_key)
    print(f"[INFO] Total merged events: {len(all_events)}")
    print(f"[INFO] Processing in chronological order...")
    print()

    # --- Process sequentially ---
    processed = 0
    for pc_time, parsed, label in all_events:
        if label == "MASTER":
            handle_master_parsed(parsed)
        else:
            handle_slave_parsed(parsed)
        processed += 1

    print(f"[INFO] Processed {processed} events")

    # --- Print final status ---
    print(f"\n{'='*72}")
    print("  FINAL SYNCHRONIZATION STATUS")
    print(f"{'='*72}")
    if engine.reset_count > 0:
        print(f"  MA restarts detected: {engine.reset_count}")
        print(f"  Pending tags dropped: {engine.dropped_packets_on_reset}")
    status = engine.get_sync_status()
    for sid, st in sorted(status.items()):
        print(f"\n  Slave Anchor {sid}:")
        print(f"    Sync cycles : {st['sync_count']}")
        print(f"    LI ready    : {st['li_ready']}")
        if st['delta_k']:
            print(f"    δ_k (LI)    : {st['delta_k']:.15f}")
        if st['theta_k_us']:
            print(f"    θ_k         : {st['theta_k_us']:.3f} μs")
        if KF_ENABLED:
            print(f"    KF converged: {st['kf_converged']}")
            print(f"    KF updates  : {st['kf_updates']}")
            if st['kf_drift'] is not None:
                print(f"    KF drift    : {st['kf_drift']:.15f}")
                print(f"    KF drft_rate: {st['kf_drift_rate']:.6e}")
    print(f"\n{'='*72}")

    engine.close()

    print(f"\n  Output files:")
    print(f"    {STEP1_OUTPUT_CSV}")
    print(f"    {STEP2_OUTPUT_CSV}")
    print(f"    {KF_OUTPUT_CSV}")
    print()

if __name__ == "__main__":
    main()