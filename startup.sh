# this file will run inside the docker container and should have all the commands needed to run ros

#this runs bo
ros2 launch demo_nodes_cpp listener&
ros2 launch demo_nodes_cpp talker&
