# KRTI 2025 Quadcopter Simulation Stack

Dokumentasi arsitektur komunikasi data dan orkestrasi sistem navigasi otonom berbasis **EGO-Planner Swarm** dan **VINS-Fusion**.

## 1. Persiapan & Instalasi (Setup)

Pastikan kamu berada di root workspace (`~/ego_ws`):

```bash
cd ~/ego_ws
# Compile workspace
catkin_make
# Tambahkan source ke bashrc agar otomatis terdeteksi (Hanya sekali)
echo "source ~/ego_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Cara Menjalankan Simulasi (4 Terminal)

Buka 4 terminal terpisah dan jalankan perintah berikut secara berurutan:

### Terminal 1: Simulasi Gazebo & MAVROS
Menjalankan environment arena, model drone, VINS-Fusion, dan EGO-Planner.
```bash
sim_bringup
```

### Terminal 2: Flight Control Logic (SITL)
Menjalankan firmware ArduPilot di latar belakang.
```bash
startsitl
```

### Terminal 3: Mission Control (Main Program)
Menjalankan logika misi utama dan wrapper `DroneAPI`.
```bash
rosrun krti2024_pi main.py
```

### Terminal 4: Visualisasi & Debugging
Membuka GUI untuk memantau data sensor atau status sistem.
```bash
rqt
```
*Note: Pilih menu **Plugins -> Visualization -> Plot** untuk grafik atau **Image View** untuk melihat kamera drone.*

---

## 3. Stack Arsitektur & Aliran Data
*   **Physics Engine**: Gazebo (Arena 2023 world).
*   **Localization (VIO)**: VINS-Fusion -> MAVROS vision_pose.
*   **Planning**: EGO-Planner Swarm (B-Spline Trajectory).
*   **Control**: DroneAPI (Yaw-to-Velocity / Face Forward).

## 4. Catatan untuk Pengembangan Real Drone (WIP)
> [!IMPORTANT]
> Sistem ini sedang dikembangkan untuk transisi ke wahana asli (Real Drone). Perhatikan hal berikut:
> *   **Localization**: Di real drone, pastikan pencahayaan cukup untuk VINS-Fusion agar tidak terjadi *tracking loss*.
> *   **Safety**: Gunakan switch RC untuk mengambil alih kendali manual (`STABILIZE` atau `LOITER`) jika terjadi anomali pada Planner.
> *   **Calibration**: Lakukan kalibrasi IMU dan magnetometer secara presisi sebelum mencoba navigasi otonom di luar ruangan.

---
*Dokumentasi ini diperbarui untuk repository krti2025-simulation.*
