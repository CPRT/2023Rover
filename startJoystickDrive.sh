#!/bin/bash

bluetoothctl power on

dir=/home/cprt/2023Rover

source /opt/ros/humble/setup.bash

source $dir/install/local_setup.bash

ros2 launch drive RoverDriveWithJoystick.launch.py
