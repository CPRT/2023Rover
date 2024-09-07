#!/bin/bash


# List of directories to exclude
# Only exclude third party code
excluded_dirs=("ros_phoenix_humble" "ouster-ros" "spatio_temporal_voxel_layer")

# Function to find files excluding certain directories
find_files_excluding_dirs() {
    local extensions=("$@")
    
    # Remove the directory list from the extensions array
    local ext_list=("${extensions[@]:0:${#extensions[@]}-1}")
    local last_ext=${extensions[-1]}

    # Build the find command dynamically
    local find_cmd="find . -type d \( "
    for dir in "${excluded_dirs[@]}"; do
        find_cmd+=" -path ./$dir -o"
    done

    # Remove the trailing '-o'
    find_cmd=${find_cmd%-o}

    find_cmd+=" \) -prune -o \( "

    # Build the name pattern for multiple extensions
    local ext_pattern=""
    for ext in "${ext_list[@]}"; do
        ext_pattern+=" -name '*.$ext' -o"
    done

    # Remove the trailing '-o'
    ext_pattern=${ext_pattern%-o}

    find_cmd+="$ext_pattern \) -print"

    # Execute the find command
    eval "$find_cmd"
}


python_files=find_files_excluding_dirs "py"

black $python_files

cpp_files=find_files_excluding_dirs "h" "hpp" "cpp"

clang-format -i -style=file $cpp_files

rosdep install --from-paths src -r -y

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install

clang-tidy -p ./build $cpp_files $INCLUDE_PATHS || exit 1

source install/local_setup.bash