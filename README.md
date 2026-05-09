# KRTI 2025 Quadcopter Simulation Stack

Dokumentasi arsitektur komunikasi data dan orkestrasi sistem navigasi otonom berbasis **EGO-Planner Swarm** dan **VINS-Fusion**.

## 1. Instalasi Dari Nol (Fresh Installation)

Ikuti langkah-langkah ini jika kamu ingin menginstall sistem ini di komputer baru:

```bash
# 1. Buat folder workspace
mkdir -p ~/ego_ws/src
cd ~/ego_ws/src

# 2. Clone repository ini
git clone https://github.com/mochshultan/krti2025-simulation.git .

# 3. Inisialisasi & Build Workspace
cd ~/ego_ws
catkin_make

# 4. Daftarkan ke sistem (Bashrc)
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

## 3. Stack Arsitektur & Aliran Data (Technical Flow)

### A. Lokalisasi (VIO Pipeline)
1.  **Sensor Input**: Gazebo mempublikasikan `image_raw` dan `imu/data` dari model drone.
2.  **Estimation**: VINS-Fusion memproses data sensor untuk menghasilkan estimasi posisi (`/vins_estimator/odometry`).
3.  **Bridge**: Node `vins_to_mavros.py` menangkap output VINS, melakukan transformasi koordinat ke frame `world`, dan mengirimkannya ke `/mavros/vision_pose/pose`.
4.  **EKF Fusion**: ArduPilot memadukan data *vision* tersebut ke dalam EKF3 untuk menghasilkan `/mavros/local_position/odom` yang stabil.

### B. Perencanaan Jalur (Navigation Pipeline)
1.  **Goal Setting**: `DroneAPI` mengirimkan target posisi ke topik `/move_base_simple/goal` (Type: `geometry_msgs/PoseStamped`).
2.  **Obstacle Sensing**: EGO-Planner menerima data `depth/points` dari Gazebo untuk membangun *ESDF Map* secara lokal.
3.  **Optimization**: Planner menghitung jalur optimal berbasis B-Spline yang menghindari rintangan dan mempublikasikannya ke `/drone_0_planning/pos_cmd`.
4.  **Control Loop**: `DroneAPI` menangkap `pos_cmd`, menghitung **Yaw-to-Velocity** (Face Forward), dan mengirimkan perintah `PositionTarget` ke MAVROS `/mavros/setpoint_raw/local`.

## 4. Protokol Komunikasi & TF Tree

| Komponen | Topik Utama | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- | :--- |
| **Localization** | `/mavros/local_position/odom` | `nav_msgs/Odometry` | Umpan balik posisi (VIO-based) |
| **Sensing** | `/iris/front_camera/depth/points` | `sensor_msgs/PointCloud2` | Data awan titik untuk deteksi rintangan |
| **Planning** | `/drone_0_planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | Output jalur dari EGO Planner |
| **Setpoint** | `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Perintah final ke Flight Controller |

### TF Coordinate Hierarchy
*   **Root**: `world` → `map` → `odom` → `base_link`
*   **Camera**: `base_link` → `depth_camera_link` (Static Offset: x=0.1, z=0.05)

## 5. Catatan untuk Pengembangan Real Drone (WIP)
> [!IMPORTANT]
> Sistem ini sedang dikembangkan untuk transisi ke wahana asli (Real Drone). Perhatikan hal berikut:
> *   **Localization**: Di real drone, pastikan pencahayaan cukup untuk VINS-Fusion agar tidak terjadi *tracking loss*.
> *   **Safety**: Gunakan switch RC untuk mengambil alih kendali manual (`STABILIZE` atau `LOITER`) jika terjadi anomali pada Planner.
> *   **Calibration**: Lakukan kalibrasi IMU dan magnetometer secara presisi sebelum mencoba navigasi otonom di luar ruangan.

---

## 6. Workflow Pengembangan (Update Kode)

Setiap kali kamu melakukan perubahan kode dan ingin mengunggahnya ke GitHub:

```bash
cd ~/ego_ws/src
git add .
git commit -m "Deskripsi perubahan kamu"
git push origin main
```

---
*Dokumentasi ini diperbarui untuk repository krti2025-simulation.*
