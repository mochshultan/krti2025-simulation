#!/bin/bash
# Script untuk menjalankan vision dengan konfigurasi yang benar

# Source workspace
source /home/hafiz/sim26/src/krti2024/devel/setup.bash

# Tampilkan parameter yang akan digunakan
echo "========================================="
echo "Vision Configuration:"
echo "========================================="
rosparam get /vision/use_sim 2>/dev/null || echo "use_sim: NOT SET (will use from launch file)"
echo "========================================="

# Jalankan launch file
roslaunch krti2023_pi krti2023.launch
