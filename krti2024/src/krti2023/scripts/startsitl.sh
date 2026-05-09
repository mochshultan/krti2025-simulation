#!/bin/bash
# cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out 
# you can add mavproxy args here too
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --out=udp:127.0.0.1:14550 
# udpout to windows from WSL2  