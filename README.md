git clone https://github.com/CPRT/2023Rover.git

cd 2023Rover

git checkout Arm-Simulation

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash 

ros2 launch rover_arm_config demo.launch.py

takes maybe a minute or 2 to get the planner running, if you see a ton of green text saying its ready in the console try moving around the ball and pressing plan/ plan and execute
