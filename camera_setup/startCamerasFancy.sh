#!/usr/bin/bash
shopt -s extglob

start="gst-launch-1.0"

end="mpegtsmux name=mux alignment=7 ! srtsink uri=srt://:9000 latency=200 sync=true"

encoder="nvvidconv ! nvv4l2h265enc bitrate=5000000 control-rate=0 maxperf-enable=true insert-sps-pps=true ! mux."

nightVision="v4l2src device=/dev/v4l/by-id/usb-e-con_systems_See3CAM_CU27_3B1519112B010900-video-index0 ! $encoder"

SecondaryTopCam="v4l2src device=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0 ! video/x-raw, height=480, width=640, framerate=30/1 ! $encoder"


#
# Define argument variables
#
ENABLE_NIGHTVISION_CAM=0
ENABLE_SECONDARY_TOP_CAM=0

#
# Parse arguments
#
while (( $# > 0 )) ; do
    case $1 in 
        -n | --night-vision)
            ENABLE_NIGHTVISION_CAM=1
            ;;
        -t | --top-cam)
            ENABLE_SECONDARY_TOP_CAM=1
            ;;
        -*) printf >&2 'Unknown option %s\n' "$1" ; exit 1 ;;
        *) break ;;
    esac
    shift
done

#
# Print arguments
#
echo "ENABLE_NIGHTVISION_CAM: $ENABLE_NIGHTVISION_CAM"
echo "ENABLE_SECONDARY_TOP_CAM: $ENABLE_SECONDARY_TOP_CAM"

# 
# Create Camera Gstreamer settings
#
GST_CAM_SETTINGS=""

if [ "$ENABLE_NIGHTVISION_CAM" == "1" ]; then
    GST_CAM_SETTINGS="$GST_CAM_SETTINGS $nightVision"
    echo "Adding night vision camera"
fi

if [ "$ENABLE_SECONDARY_TOP_CAM" == "1" ]; then
    GST_CAM_SETTINGS="$GST_CAM_SETTINGS $SecondaryTopCam"
    echo "Adding secondary top camera"
fi

#
# Check for empty options
#
if [ "$GST_CAM_SETTINGS" == "" ]; then
    echo "Must specify at least one camera in the options"
    exit 2
fi


#
# Run gstreamer
#
cmd="$start $GST_CAM_SETTINGS $end"
eval ${cmd}

