#!/bin/bash 
shopt -s extglob

# INPUT
INPUT_SERVICES_DIR="ffmpeg_piping_services"
SERVICE_DIR_NAME="active_services"

# CONVERT INPUT
SRC_DIR=$(cd "$INPUT_SERVICES_DIR/.."; pwd)
SERVICE_DIR="$SRC_DIR/$SERVICE_DIR_NAME"
INPUT_SERVICES_DIR="$SRC_DIR/$INPUT_SERVICES_DIR"

SERVICE_PATHS=$(echo "$INPUT_SERVICES_DIR"/*.service)
SERVICE_NAMES=$(basename --multiple "$INPUT_SERVICES_DIR"/*.service | tr '\n' ' ')

PYTHON_INTERP="$SRC_DIR/ffmpeg_piping_scripts/.ffmpeg_venv/bin/python"
PYTHON_SCRIPT="$SRC_DIR/ffmpeg_piping_scripts/pipe_ffmpeg_to_v4l2loopback.py"

# Display input
echo "INPUTS:"
echo "  SRC_DIR:            $SRC_DIR"
echo "  INPUT_SERVICES_DIR: $INPUT_SERVICES_DIR"
echo "  SERVICE_DIR:        $SERVICE_DIR"
echo "  SERVICE_FILEPATHS:  $SERVICE_PATHS" 
echo "  SERVICE_NAMES: ${SERVICE_NAMES}"
echo "  SERVICE_PATHS: ${SERVICE_PATHS}"
echo
echo "  PYTHON_INTERP: ${PYTHON_INTERP}"
echo "  PYTHON_SCRIPT: ${PYTHON_SCRIPT}"
echo

echo "~~ Reload services"
sudo systemctl daemon-reload
echo

echo "~~ Stop and disable services"
sudo systemctl stop ${SERVICE_NAMES}
sudo systemctl disable ${SERVICE_NAMES}
echo

rm -rf "$SERVICE_DIR"
mkdir -p "$SERVICE_DIR"
cp -r "$INPUT_SERVICES_DIR"/*.service "$SERVICE_DIR"

echo "~~ Copy services from input service directory to $SERVICE_DIR_NAME"
SERVICE_PATHS=$(echo "$SERVICE_DIR"/*.service)
SERVICE_NAMES=$(basename --multiple "$SERVICE_DIR"/*.service | tr '\n' ' ')
echo "  SERVICE_NAMES: ${SERVICE_NAMES}"
echo "  SERVICE_PATHS: ${SERVICE_PATHS}"


# Modify the location of the python venv interpeter and the python script in the services
echo "~~ Update the location of the python venv interpeter and script in the ExecStart in the services"
sed -i "s#REPLACE_ME_AUTOMATICALLY#$PYTHON_INTERP -u $PYTHON_SCRIPT#" $SERVICE_PATHS
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

