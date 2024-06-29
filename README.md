# Carleton Planetary Robotics Team 2023 Rover Repository
toDo add description

## To build:
1. Clone the repository
   * $ git clone --recurse-submodules git@github.com:CPRT/2023Rover.git
2. Use rosdep to install the dependencies
   * $ rosdep rosdep install --from-paths src -r -y
3. Use colcon to build the project
   * $ colcon build --symlink-install --continue-on-error
     * Currently the gazebo package will fail untill we port over to gazebo ignition
     * Ignore warnings for the time being, please report any errors in the build process
