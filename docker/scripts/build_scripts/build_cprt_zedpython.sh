#!/usr/bin/env bash

# set -x

##########################################################
### Constants 
##########################################################

PLATFORM="$(uname -m)"
IMAGE_NAME="cprt_zed_python:2023Rover"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_NAME=$(basename "$0")
WORKSPACE_DIR=$( cd "$SCRIPT_DIR/../../.." && pwd)
WORKSPACE_NAME=$(basename "$WORKSPACE_DIR")
DOCKERFILE_DIR="$WORKSPACE_DIR/docker"

# echo "PLATFORM: $PLATFORM"
# echo "IMAGE_NAME: $IMAGE_NAME"
# echo
# echo "SCRIPT_DIR: $SCRIPT_DIR"
# echo "SCRIPT_NAME: $SCRIPT_NAME"
# echo "WORKSPACE_DIR: $WORKSPACE_DIR"
# echo "WORKSPACE_NAME: $WORKSPACE_NAME"
# echo "DOCKERFILE_DIR: $DOCKERFILE_DIR"

##########################################################
### Safety Checks 
##########################################################

if [ ! -d "$DOCKERFILE_DIR" ]; then
    echo "Missing docker dir at $DOCKERFILE_DIR"
    exit 1
fi

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Missing workspace dir at $WORKSPACE_DIR"
    exit 2
fi

##########################################################
### Source helpful scripts
##########################################################

if [ ! -f "$DOCKERFILE_DIR/scripts/sourced_scripts/bash_functions.sh" ]; then
    echo "Cannot find bash_functions.sh file"
    exit 3
else
    # shellcheck disable=SC1091
    source "$DOCKERFILE_DIR/scripts/sourced_scripts/bash_functions.sh" || exit 4
fi

##########################################################
### Docker Build
##########################################################

cd "$DOCKERFILE_DIR" || exit 20

echo
if [ "$PLATFORM" == "aarch64" ]; then
    print_info "Selecting cuda jetpack image for base image"
    initial_image="nvcr.io/nvidia/l4t-cuda:12.2.12-devel"
else
    print_info "Selecting tritonserver (x86 cuda image) for base image"
    initial_image="nvcr.io/nvidia/tritonserver:23.10-py3"
fi

print_docker_build "Building Dockerfile.ros2_humble as ros2_humble from $initial_image"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.ros2_humble \
    --network host \
    --build-arg "BASE_IMAGE=$initial_image" \
    -t ros2_humble:cprt_ros2_2023Rover . \
    || exit 21

print_docker_build "Building Dockerfile.user as user from ros2_humble"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.user \
    --network host \
    --build-arg BASE_IMAGE=ros2_humble:cprt_ros2_2023Rover \
    -t user:cprt_ros2_2023Rover . \
    || exit 22

print_docker_build "Building Dockerfile.zed as zed from user"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.user \
    --network host \
    --build-arg BASE_IMAGE=user:cprt_ros2_2023Rover \
    -t zed:cprt_ros2_2023Rover . \
    || exit 23

print_docker_build "Building Dockerfile.cprt_2023Rover as final image named $IMAGE_NAME from zed"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.cprt_2023Rover \
    --network host \
    --build-context "cprt_repo=$WORKSPACE_DIR" \
    --build-arg BASE_IMAGE=zed:cprt_ros2_2023Rover \
    -t "$IMAGE_NAME" . \
    || exit 24


print_info "Built image $IMAGE_NAME successfully"
