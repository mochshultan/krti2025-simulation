# KRTI 2025 Quadcopter Simulation Stack

Dokumentasi arsitektur komunikasi data dan orkestrasi sistem navigasi otonom berbasis **EGO-Planner Swarm** dan **VINS-Fusion**.

## 1. Stack Arsitektur
Sistem ini menggunakan integrasi *multi-layer* untuk simulasi drone:
*   **Physics Engine**: Gazebo (Arena 2023 world).
*   **Flight Stack**: ArduPilot SITL (Firmware ArduCopter).
*   **Communication Bridge**: MAVROS (MAVLink to ROS conversion).
*   **Localization (VIO)**: VINS-Fusion (Visual-Inertial Odometry).
*   **Navigation & Planning**: EGO-Planner Swarm (B-Spline Trajectory Optimization).
*   **Interface**: DroneAPI (Python wrapper for mission orchestration).

## 2. Aliran Data (Data Flow)

### A. Lokalisasi (VIO Pipeline)
1.  **Sensor Input**: Gazebo mempublikasikan `image_raw` dan `imu/data` dari model drone.
2.  **Estimation**: VINS-Fusion memproses data sensor untuk menghasilkan estimasi posisi (`/vins_estimator/odometry`).
3.  **Bridge**: Node `vins_to_mavros.py` menangkap output VINS, melakukan transformasi koordinat ke frame `world`, dan mengirimkannya ke `/mavros/vision_pose/pose`.
4.  **EKF Fusion**: ArduPilot memadukan data *vision* tersebut ke dalam EKF3 untuk menghasilkan `/mavros/local_position/odom` yang stabil.

### B. Perencanaan Jalur (Navigation Pipeline)
1.  **Goal Setting**: `DroneAPI` mengirimkan target posisi ke topik `/move_base_simple/goal` (Type: `geometry_msgs/PoseStamped`).
2.  **Obstacle Sensing**: EGO-Planner menerima data `depth/image_raw` dan `depth/points` dari Gazebo untuk membangun *ESDF Map* secara lokal.
3.  **Optimization**: Planner menghitung jalur optimal berbasis B-Spline yang menghindari rintangan dan mempublikasikannya ke `/drone_0_planning/pos_cmd`.
4.  **Control Loop**: `DroneAPI` menangkap `pos_cmd`, menghitung **Yaw-to-Velocity** (Face Forward), dan mengirimkan perintah `PositionTarget` ke MAVROS `/mavros/setpoint_raw/local`.

## 3. Protokol Komunikasi (Topics & Frames)

| Komponen | Topik Utama | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- | :--- |
| **Localization** | `/mavros/local_position/odom` | `nav_msgs/Odometry` | Umpan balik posisi (VIO-based) |
| **Sensing** | `/iris/front_camera/depth/points` | `sensor_msgs/PointCloud2` | Data awan titik untuk deteksi rintangan |
| **Planning** | `/drone_0_planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | Output jalur dari EGO Planner |
| **Setpoint** | `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Perintah final ke Flight Controller |
| **Trigger** | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Trigger untuk memulai perencanaan jalur |

## 4. Transformasi Koordinat (TF Tree)
*   **Root Frame**: `world`
*   **Map Frame**: `map` (Static to `world`)
*   **Odom Frame**: `odom` (Driven by SITL/VINS)
*   **Robot Frame**: `base_link`
*   **Camera Frame**: `depth_camera_link` (Static TF dari `base_link` x:0.1m, z:0.05m)

## 5. Cara Menjalankan
1.  **SITL**: `./startsitl.sh` (In `krti2023/scripts`)
2.  **Bringup Stack**: `roslaunch krti2025_bringup sim.launch`
3.  **Mission Control**: `rosrun krti2024_pi main.py`

---
*Dokumentasi ini dibuat otomatis oleh Antigravity AI Assistant.*
