#!/bin/bash
# ============================================================
#  start_sim.sh — REFERENSI SAJA
#  
#  ⚠️  SITL WAJIB dijalankan di terminal TERPISAH!
#  ⚠️  Jangan jalankan script ini langsung.
#
#  URUTAN YANG BENAR:
#    Terminal 1:  startsitl   (atau: ~/ego_ws/src/krti2025_pi/scripts/startsitl.sh)
#                 Tunggu sampai SITL siap: "APM: EKF3 IMU0 tilt alignment complete"
#    Terminal 2:  sim_bringup (atau: roslaunch krti2025_bringup sim.launch)
# ============================================================
echo "[INFO] Jalankan SITL di Terminal 1 terlebih dahulu!"
echo "[INFO] startsitl  ->  ~/ego_ws/src/krti2025_pi/scripts/startsitl.sh"
echo ""
echo "[INFO] Setelah SITL siap, jalankan di Terminal 2:"
echo "[INFO] sim_bringup  ->  roslaunch krti2025_bringup sim.launch"