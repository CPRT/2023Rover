git clone https://github.com/CPRT/2023Rover.git

cd 2023Rover

git checkout Arm-Simulation

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash 

ros2 launch rover_arm_config demo.launch.py

takes maybe a minute or 2 to get the planner running, if you see a ton of green text saying its ready in the console try moving around the ball and pressing plan/ plan and execute



Launch simple controller:

Terminal 1:
source ros and install
ros2 launch rover_arm_config demo.launch.py

terminal 2:
ros2 run moveit_controller moveit_controller

terminal 3:
ros2 run controller_publisher controller_publisher
Start entering w, a, s, or d and press enter
z and x to go up and down
r and t to twist wrist
f and g to rotate wrist up and down
c and v to rotate wrist side to side
q to stop