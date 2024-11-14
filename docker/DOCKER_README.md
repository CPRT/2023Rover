
# Documentation for the Docker system used with the Carleton Plantary Robotics Team

For a generic ros2 node, run the following command within the scripts directory:
```
./run_docker.sh --image cprt_ros2 --container <container-name>
```

To make a specific container more permenant, duplicate the run_cprt_ros2.sh.



# Design

The Dockerfiles use the `ARG BASE_IMAGE` to define where the base image comes from. 
The bash scripts to use `--build-arg BASE_IMAGE=<base-image>` and `--target <target-image>` to input and output image names for that step. This allows flexibility in combining any Dockerfile together with bash scripts.

## Scripts and Directories

### build_scripts directory

This directory contains scripts that define the docker build commands for a specific image. 
The script name must be `"build_$IMAGE_NAME"` for it to be picked up by build_docker.sh.

### build_docker.sh

This script has a `--image <IMAGE_NAME>` argument to specify which image to build.
Relies on the scripts in the build_scripts directory to be named as `"build_$IMAGE_NAME"`

### run_docker.sh

This script has `--image <IMAGE_NAME>` and `--container <CONTAINER_NAME>` arguments.
The script original came from the Nvidia Isaac Ros2 ecosystem and is fairly intelligent.
Running the script multiple times with the same image and container names will reconnect to the same container,
allowing multiple terminals within the same container.

The script will call build_docker.sh by default and rebuild the image. Use the `--skip-image-build`/`-b` option to skip building the image and use the last cached version of the container instead.

### run_scripts directory

The directory contains simple bash scripts to define aliases for all regularly used arguments of run_docker.sh. 
This directory should be added to the PATH to make them accessable anywhere.

## Dockerfiles

### Dockerfile.cprt_2023Rover

Copies only the package.xml files from your local version of the 2023Rover repo into the docker container.
Runs rosdep install to install the nessecary packages into the container.

It only copies to package.xml files to maintain good caching. It will only invalid the rosdep build step when package.xml files are changed.

### Dockerfile.user

Creates a cprtdev user with proper permissions.

### Dockerfile.ros2_humble

Installs ros2_humble and various other basic packages.

### Dockerfile.zed

Installs the ZED SDK. Works for both x86 and aarach64.
ZED requires a CUDA enabled Nvidia GPU to function.

### Dockerfile.isaac_aarch64_cuda

Installs and sets up various packages to run any nvidia isaac package.
