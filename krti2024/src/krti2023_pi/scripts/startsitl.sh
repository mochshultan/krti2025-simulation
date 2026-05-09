#!/bin/bash
# cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out 
# you can add mavproxy args here too
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console -L UNER --out=udp:192.168.18.188:14550 --out=udp:172.18.32.1:14550 
# udpout to windows from WSL2  
