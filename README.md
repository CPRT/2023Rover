First-time instructions:

git clone https://github.com/CPRT/2023Rover.git

cd 2023Rover

git checkout Arm-Simulation

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash 

Get peak-usb-driver installed:
Step 1: Plug arm usb into computer

Step 2: Download https://www.peak-system.com/Details.114+M5331421eb48.0.html?&L=1

Step 3: Copy the 2023Rover/drivers/installnet.sh into the newly downloaded peak-driver-8.18.0 folder

Step 4: cd into the driver folder, and run "./installnet.sh"


Launch simple controller for inverse kinematics:

Terminal 1:
source ros and install
ros2 launch test_urdf_v2 demo.launch.py

terminal 2:
ros2 run moveit_controller moveit_controller

terminal 3:
ros2 run keyboard_publisher keyboard_publisher
Start entering w, a, s, or d and press enter
z and x to go up and down
r and t to twist wrist
f and g to rotate wrist up and down
c and v to rotate wrist side to side
q to stop
n to reset to original position
b + 6 numbers to set angles in degrees from orange arm in rviz
(Example command: b35 67 12 -42 5 1)

Terminal 4:
ros2 launch arm_interface keyboard.launch.py

Once RViz is running, you can pick and place the orange ball and press "plan and execute"
