#!/bin/bash

# Clear bluetooth issue
# bring each stopped job back and kill it
kill %1 %2 %3 %4 %5 %6 %7
# or just nuke all of them at once
pkill -f muselsl
bluetoothctl disconnect 00:55:DA:B8:34:01

# Clear Colcon build Logs
cd ~/rob/rob_ws
rm -rf build install log
