#!/bin/bash


# List of directories to exclude
# Only exclude third party code
excluded_dirs=( "build" \
                "install" \
                "log" \
                "ros_phoenix_humble" \
                "ouster-ros" \
                "spatio_temporal_voxel_layer" \
                "ffmpeg_piping_scripts" \
                "openvdb_vendor" \
                "ouster_sensor_msgs" \
                "interfaces" \
                "openvdb_vendor")

# Function to find files excluding certain directories
find_files_excluding_dirs() {
    local extension=("$1")
    # Build the find command dynamically
    local find_cmd="find . -type d \( "
    for dir in "${excluded_dirs[@]}"; do
        find_cmd+=" -name $dir -o"
    done

    # Remove the trailing '-o'
    find_cmd=${find_cmd%-o}

    find_cmd+=" \) -prune -o \( "
    find_cmd+=" -name '*.$extension'"
    find_cmd+=" \) -print"

    # Execute the find command
    eval "$find_cmd"
}

python_files=$(find_files_excluding_dirs "py")
python -m black $python_files || exit 1

echo "Starting to find c++ files..."

cpp_files="$(find_files_excluding_dirs "h") $(find_files_excluding_dirs "cpp") $(find_files_excluding_dirs "hpp")"

echo "Found c++ files"

echo "Starting clang-format..."

clang-format -i -style=file $cpp_files || exit 1

echo "Finished clang-format"

echo "Starting rosdep install..."

rosdep install --ignore-src --from-paths src -r -y || exit 1

echo "Finished rosdep install"

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install || exit 1

#TODO: Debug why clang tidy can't find std:: headers
# clang-tidy -p ./build $cpp_files $INCLUDE_PATHS || exit 1

source install/local_setup.bash