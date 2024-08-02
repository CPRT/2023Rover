#!/bin/bash 
shopt -s extglob

# INPUT
DIR="ffmpeg_piping_services"

# CONVERT INPUT
SRC_DIR=$(cd "$(dirname "$DIR")"; pwd)
SERVICE_DIR="$SRC_DIR/$(basename "$DIR")"

SERVICE_PATHS=$(echo "$SERVICE_DIR"/*.service)
SERVICE_NAMES=$(basename --multiple "$SERVICE_DIR"/*.service | tr '\n' ' ')

PYTHON_INTERP="$SRC_DIR/ffmpeg_piping_scripts/.ffmpeg_venv/bin/python"
PYTHON_SCRIPT="$SRC_DIR/ffmpeg_piping_scripts/pipe_ffmpeg_to_v4l2loopback.py"

# Display input
echo "INPUTS:"
echo "  SERVICE_DIR: $SERVICE_DIR"
echo "  SERVICE_FILEPATHS: $SERVICE_PATHS" 
echo "  SERVICE_NAMES: ${SERVICE_NAMES}"
echo
echo "  PYTHON_INTERP: ${PYTHON_INTERP}"
echo "  PYTHON_SCRIPT: ${PYTHON_SCRIPT}"
echo

# Modify the location of python and the script in the services
echo "~~ Update the location of the python interpeter and script in the ExecStart in the services"
sed -i "s#ExecStart.*py#ExecStart=$PYTHON_INTERP -u $PYTHON_SCRIPT#" $SERVICE_PATHS
echo

echo "~~ Reload services"
sudo systemctl daemon-reload
echo

# Copy and enable the services
echo "~~ Stop and disable services"
sudo systemctl stop ${SERVICE_NAMES}
sudo systemctl disable ${SERVICE_NAMES}
echo

echo "~~ Link services to files in the repo"
sudo systemctl link ${SERVICE_PATHS}
echo

echo "~~ Enable services"
sudo systemctl enable ${SERVICE_NAMES}
echo

echo "~~ Start services"
sudo systemctl start ${SERVICE_NAMES}
echo

