#!/bin/bash 
shopt -s extglob

# INPUT
DIR="ffmpeg_piping_services"

# CONVERT INPUT
SRC_DIR=$(cd "$(dirname "$DIR")"; pwd)
SERVICE_DIR="$SRC_DIR/$(basename "$DIR")"

SERVICE_NAMES=$(basename --multiple "$SERVICE_DIR"/*.service | tr '\n' ' ')

# Display input
echo "INPUTS:"
echo "  SERVICE_DIR: $SERVICE_DIR"
echo "  SERVICE_NAMES: ${SERVICE_NAMES}"
echo

echo "~~ Stop services"
sudo systemctl stop ${SERVICE_NAMES}
echo
