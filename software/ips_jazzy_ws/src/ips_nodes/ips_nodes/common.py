"""Shared utilities for all IPS nodes.

QoS profiles + system constants. Anchor numbering matches the thesis:
    ID 1 = MC (Master Clock — TX-only CCP source)
    ID 2 = MA (Master Anchor — receives blinks; reference A1 in Chan)
    ID 3 = SA3
    ID 4 = SA4
    ID 5 = SA5
"""

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


# -----------------------------------------------------------------------------
# QoS profiles
# -----------------------------------------------------------------------------

QOS_SENSOR_BEST_EFFORT_DEEP = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,
    durability=QoSDurabilityPolicy.VOLATILE,
)

QOS_SENSOR_BEST_EFFORT = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
    durability=QoSDurabilityPolicy.VOLATILE,
)

QOS_STATE_RELIABLE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=20,
    durability=QoSDurabilityPolicy.VOLATILE,
)

QOS_LATCHED_CONFIG = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


# -----------------------------------------------------------------------------
# Anchor IDs (matches thesis convention)
# -----------------------------------------------------------------------------

ANCHOR_MC  = 1   # Master Clock (TX-only)
ANCHOR_MA  = 2   # Master Anchor (RX, reference A1 in Chan)
ANCHOR_SA3 = 3
ANCHOR_SA4 = 4
ANCHOR_SA5 = 5

# Anchor IDs that receive blinks (used by position_solver).
# MUST be in this order — index 0 is the Chan reference.
RX_ANCHORS_ORDERED = (ANCHOR_MA, ANCHOR_SA3, ANCHOR_SA4, ANCHOR_SA5)


# -----------------------------------------------------------------------------
# UWB report types (must match ips_msgs/UwbAnchorReport constants)
# -----------------------------------------------------------------------------

REPORT_TYPE_MASTER_CLOCK = 0
REPORT_TYPE_SLAVE_MASTER = 1
REPORT_TYPE_SLAVE_TAG    = 2


# -----------------------------------------------------------------------------
# Session event kinds (must match ips_msgs/SessionEvent constants)
# -----------------------------------------------------------------------------

EVENT_HELLO     = 0
EVENT_RESET     = 1
EVENT_HEARTBEAT = 2
EVENT_AUTO_RESTART = 3


# -----------------------------------------------------------------------------
# Networking + timing
# -----------------------------------------------------------------------------

DEFAULT_UDP_PORT = 5555
DW1000_CLOCK_HZ = 499.2e6 * 128.0
DTU_S = 1.0 / DW1000_CLOCK_HZ
SPEED_OF_LIGHT = 299_792_458.0


# -----------------------------------------------------------------------------
# IMU (tag BNO055) — Tingkat 1 ingest
# -----------------------------------------------------------------------------

TAG_ID = 1                  # default tag id (firmware TAG_ID)
IMU_RATE_HZ = 20.0          # IMU_INTERVAL_MS=50 di firmware
IMU_TOPIC = '/imu/raw'
# Unit data mentah (apa adanya, tanpa konversi):
#   euler  : derajat
#   gyro   : rad/s
#   accel  : m/s² (linear, gravity-removed, body frame)
