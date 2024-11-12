#!/usr/bin/env bash

# set -x

##########################################################
### Constants 
##########################################################


SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_NAME=$(basename "$0")

print_constants()
{
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo "Constants used by script called $SCRIPT_NAME"
    echo ""
    echo "Script Details:"
    echo "    SCRIPT_DIR: $SCRIPT_DIR"
    echo "    SCRIPT_NAME: $SCRIPT_NAME"
    echo ""
    echo "Arguments:"
    echo "    IMAGE_NAME: $IMAGE_NAME"
    echo "    BUILD_SCRIPT: $BUILD_SCRIPT"
    echo ""
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
}

#######################################################################################
### Argument Parser
#######################################################################################

usage()
{
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo "Builds Dockerfile for Isaac Ros2"
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo
    echo "options:"
    echo "    -i or --image <image_name>   Specify the base image name to use."
    echo "    -p or --print-constants      Print the script's constants for debugging purposes"
    echo "    -h or --help                 Print this help."
    echo
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo
}

SKIP_IMAGE_BUILD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--image)
            IMAGE_NAME="$2"
            IMAGE_NAME=$(echo "$IMAGE_NAME" | cut -f1 -d":")
            BUILD_SCRIPT="$SCRIPT_DIR/build_scripts/build_$IMAGE_NAME.sh"
            shift 2
            ;;
        -p|--print-constants)
            print_constants
            exit 0
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Invalid option: $1"
            echo
            usage
            exit 1
            ;;
    esac
done

#######################################################################################
### Safety Checks
#######################################################################################

if [ -z "$IMAGE_NAME" ]; then
    echo "Missing image name. Please specify an image name with -i or --image"
    exit 2
fi

if [ ! -f "$BUILD_SCRIPT" ]; then
    echo "Invalid image name. Cannot find image $IMAGE_NAME build script at:"
    echo "$BUILD_SCRIPT"
    exit 3
fi

#######################################################################################
### Run the build script
#######################################################################################

chmod +x "$BUILD_SCRIPT"
bash "$BUILD_SCRIPT"