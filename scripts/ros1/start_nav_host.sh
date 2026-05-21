#!/bin/bash
pkill -9 -f bridge_and_control 2>/dev/null
sleep 1
rm -f /dev/shm/cyclonedds* /dev/shm/dds*
nohup python3 $HOME/bridge_and_control.py > /tmp/bridge_control.log 2>&1 &
sleep 6
cat /tmp/bridge_control.log
