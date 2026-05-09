# KRTI 2025 Bringup

Package ini berfungsi sebagai pusat *bringup* untuk drone KRTI 2025, baik dalam mode simulasi (SITL + Gazebo) maupun mode real (Companion PC + FCU).

## Mode Simulasi (Companion PC Headless / dengan Gazebo)

Arsitektur simulasi memisahkan proses SITL (ArduPilot) dengan MAVROS dan visualisasi. Hal ini memungkinkan simulasi berjalan lebih stabil.

**Langkah Menjalankan Simulasi:**

1. **Terminal 1: Jalankan SITL**
   ```bash
   startsitl
   ```
   *(Alias dari: `~/ego_ws/src/krti2025_pi/scripts/startsitl.sh`)*
   
   Tunggu hingga muncul pesan `APM: EKF3 IMU0 tilt alignment complete` yang menandakan estimasi state drone sudah siap.

2. **Terminal 2: Jalankan Bringup Simulasi**
   ```bash
   sim_bringup
   ```
   *(Alias dari: `roslaunch krti2025_bringup sim.launch`)*
   
   Ini akan meluncurkan:
   - Gazebo (dengan model drone)
   - MAVROS (koneksi via UDP)
   - VINS-Fusion (Opsional, Estimasi VIO)
   - EGO-Planner (Perencanaan Trajectory)
   - RVIZ (Visualisasi)

## Mode Real (Companion PC)

Pada mode real, drone dihubungkan ke Companion PC via serial USB/UART. 

**Langkah Menjalankan Mode Real:**

1. Pastikan FCU terhubung ke Companion PC.
2. Pastikan VINS-Fusion telah dikalibrasi (kamera dan IMU).
3. **Terminal 1: Jalankan Bringup Real**
   ```bash
   real_bringup
   ```
   *(Alias dari: `roslaunch krti2025_bringup real.launch`)*
   
   Ini akan meluncurkan:
   - MAVROS (koneksi via Serial/USB `ttyACM0` atau `ttyAMA0`)
   - VINS-Fusion (Estimasi VIO menggunakan kamera real)
   - EGO-Planner (Perencanaan Trajectory)
   - Node Utama (Mission krti2025_pi)

## Catatan Konfigurasi VINS-Fusion

Konfigurasi VINS-Fusion untuk KRTI 2025 dapat ditemukan di direktori `src/VINS-Fusion/config/krti2025/`.
- `krti_vins_config.yaml`: Konfigurasi utama, ekstrinsik, dan noise IMU.
- `krti_cam0.yaml`: Parameter intrinsik kamera.

⚠️ **Penting:** Lakukan kalibrasi intrinsik dan ekstrinsik kamera dengan kalibrator ROS atau Kalibr sebelum terbang di lingkungan nyata!