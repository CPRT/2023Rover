#!/usr/bin/env bash

# set -x

##########################################################
### Constants 
##########################################################

PLATFORM="$(uname -m)"
IMAGE_NAME="erik_elevationmapping_testing:2023Rover"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_NAME=$(basename "$0")
WORKSPACE_DIR=$( cd "$SCRIPT_DIR/../../.." && pwd)
WORKSPACE_NAME=$(basename "$WORKSPACE_DIR")
DOCKERFILE_DIR="$WORKSPACE_DIR/docker"

EM_WORKSPACE="$WORKSPACE_DIR/git_ignored_trial_repos/elevation_mapping_ws2"
EM_REPO_NAME="elevation_mapping_ros2"
KINDR_REPO_NAME="kindr_ros"


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

if [ ! -d "$EM_WORKSPACE" ]; then
    echo "Missing elevation_mapping_ws workspace at $EM_WORKSPACE"
    echo "Please create the workspace with the following command:"
    echo "mkdir -p 2023Rover/git_ignored_trial_repos/elevation_mapping_ws/src"
    exit 99
fi

if [ ! -d "$EM_WORKSPACE/src/$EM_REPO_NAME" ]; then
    echo "Missing elevation_mapping_ros2 repo at $EM_WORKSPACE/src/$EM_REPO_NAME"
    echo "Please clone the elevation_mapping_ros2 repo to this location using the following command:"
    echo "cd 2023Rover/git_ignored_trial_repos/elevation_mapping_ws/src && git clone --recurse-submodules https://github.com/norlab-ulaval/elevation_mapping_ros2.git"
fi

if [ ! -d "$EM_WORKSPACE/src/$KINDR_REPO_NAME" ]; then
    echo "Missing kindr ros repo at $EM_WORKSPACE/src/$KINDR_REPO_NAME"
    echo "Please clone the kindr repo to this location using the following command:"
    echo "cd 2023Rover/git_ignored_trial_repos/elevation_mapping_ws/src && git clone git@github.com:SivertHavso/kindr_ros.git && cd kindr_ros && git checkout ros2"
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
    print_info "Selecting jetson jetpack image for base image"
    initial_image="nvcr.io/nvidia/l4t-jetpack:r36.4.0"
else
    print_info "Selecting ubuntu:22.04 image for base image"
    initial_image="ubuntu:22.04"
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
docker build -f "$DOCKERFILE_DIR"/Dockerfile.zed \
    --network host \
    --build-arg BASE_IMAGE=user:cprt_ros2_2023Rover \
    -t zed:cprt_ros2_2023Rover . \
    || exit 23

print_docker_build "Building Dockerfile.cprt_2023Rover as 2023rover_repo from zed"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.cprt_2023Rover \
    --network host \
    --build-context "cprt_repo=$WORKSPACE_DIR" \
    --build-arg BASE_IMAGE=zed:cprt_ros2_2023Rover \
    -t 2023rover_repo:cprt_ros2_2023Rover . \
    || exit 23

print_docker_build "Building Dockerfile.elevation_map_testing as final image named $IMAGE_NAME from 2023rover_repo"
docker build -f "$DOCKERFILE_DIR"/Dockerfile.elevation_map_testing \
    --network host \
    --build-context "em_repo=$EM_WORKSPACE" \
    --build-arg BASE_IMAGE=2023rover_repo:cprt_ros2_2023Rover \
    -t "$IMAGE_NAME" . \
    || exit 24


print_info "Built image $IMAGE_NAME successfully"
