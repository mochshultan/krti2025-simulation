1. **mkdir -p /nama-workspace-kamu/src**
2. **cd /nama-workspace-kamu/**
3. **catkin_make**
4. **source devel/setup.bash**
5. **cd src/**
6. **git clone https://github.com/Akasasura/krti2024_pi.git**
7. **cd ..**
8. coba **catkin_make** lagi, sebelum itu **sudo apt install catkin**
9. jika gagal ganti packages di file:
   a. **package.xml**
   b. main.pi bagian **krti2024_pi.msg**, dan **krti2024_pi.srv**, sesuaikan dengan nama package workspace
   c. **CMakeLists.txt** di bagian **project(krti2023_pi)**
   d. **setup.py**
10. **rm -rf devel/ build/** pada dalam nama_workspace-kamu
11. catkin build
12. source devel/setup.bash
13. cek package dengan **rospack find nama-workspace**
14. jika tidak ada, coba masuk ke src/ lalu jalankan **echo $ROS_PACKAGE_PATH**
15. WES YA SEMOGA LANCAR DAN BERHASIL
16. GA BERHASIL, TANYA


Cara kilat instalasi ROS
1. Download dulu file instalasi ROS yang .sh
2. chmod a+x tahapan_setup_ros_full.sh
3. sudo ./tahapan_setup_ros_full.sh
