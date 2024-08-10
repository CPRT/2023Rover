# How to run Dug for a task:

## To run the joystick you need a laptop with the Rover code cloned and built. Plug in the joystick to this laptop:
cd /path/to/2023Rover
source install/setup.bash
ros2 launch drive JoystickLaunch.launch.py


## To start the wheel motor controllers:
ssh cprt@192.168.0.55
cd 2023Rover
source install/setup.bash
drivewithoutjoystick ros2 launch drive roverdrivewithoutjoystick.launch.py


# Launch the websocket to publish data for the dashboard
ssh cprt@192.168.0.55
cd 2023Rover
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml


## To start the GPS:
ssh cprt@192.168.0.55
cd 2023Rover
source install/setup.bash
ros2 launch rover_localization gps.launch.py



## To start streaming the cameras. -n for nightvision camera, -a for arm/science camera, -t for top chassis camera
ssh cprt@192.168.0.55
cd 2023Rover/camera_setup
./startCamerasFancy.sh -n -a -t

To view the cameras, Conner has VLC setup for it. Mat can run one of them with OBS if needed.


## To use anything with CAN such as the arm: (do this only once per boot)
ssh cprt@192.168.0.55
cd 2023Rover
./enablecan.sh



