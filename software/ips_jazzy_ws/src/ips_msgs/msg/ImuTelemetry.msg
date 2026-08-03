# ImuTelemetry.msg
# Telemetri IMU mentah dari tag (BNO055), data APA ADANYA — tanpa konversi.
# Format paket firmware tag:
#   $,<seq>,<ms>,<blink>,YAW,PITCH,ROLL,QW,QX,QY,QZ,GX,GY,GZ,AX,AY,AZ
#
# Revisi (Tingkat 1 ingest): mengikuti firmware tag terbaru.
#   - TAMBAH: yaw/pitch/roll (Euler deg), blink (UWB anchor untuk alignment)
#   - quaternion: dari getQuat() native BNO055 (fusi)
#   - HAPUS: magnetic_field (firmware tidak mengirimnya)
std_msgs/Header header

uint8  tag_id
uint32 seq
uint32 blink                  # nomor blink UWB terakhir saat sampel IMU diambil
                              # (jangkar time-alignment IMU<->posisi UWB; lihat README)

# Orientasi Euler (derajat) — sudah di-remap di firmware (mount berdiri 90°)
# Catatan firmware: euler.x->YAW, euler.z->PITCH, euler.y->ROLL
float64 yaw_deg
float64 pitch_deg
float64 roll_deg

# Orientasi quaternion (fusi native BNO055, getQuat())
float64 quat_w
float64 quat_x
float64 quat_y
float64 quat_z

# Gyroscope (rad/s) dan linear acceleration (m/s², gravity-removed, body frame).
# Keduanya APA ADANYA dari BNO055 — tidak ada konversi unit / rotasi.
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration

uint32 tag_time_ms            # millis() internal tag (timestamp sumber)
