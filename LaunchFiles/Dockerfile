FROM osrf/ros:humble-desktop

# Name this initial stage "builder"
FROM osrf/ros:humble-desktop AS builder
ARG DEBIAN_FRONTEND=noninteractive
RUN pip3 install opencv-python==4.1.2.30  
COPY . .
WORKDIR .

# Create a new stage for our output container
FROM osrf/ros:humble-desktop
WORKDIR .
# Copy the built binary over from previous stage
COPY test_launch.py .
# Run the binary in the new container
CMD ros2 launch test_launch.py
#CMD ros2 run demo_nodes_cpp listener
