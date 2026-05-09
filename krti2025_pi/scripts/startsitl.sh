#!/bin/bash
# cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out 
## sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=udp:127.0.0.1:14550
# you can add mavproxy args here too
cd ~/ardupilot/ArduCopter/ && ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14555
# udpout to windows from WSL2  
