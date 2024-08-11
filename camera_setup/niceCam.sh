#!/usr/bin/bash
shopt -s extglob

targetIp="192.168.1.213"

start="gst-launch-1.0"

end="mpegtsmux name=mux alignment=7 ! rtpmp2tpay ! udpsink host=$targetIp port=9000 sync=false"

encoder="nvvidconv ! nvv4l2h265enc bitrate=5000000 control-rate=0 maxperf-enable=true insert-sps-pps=true profile=1 qos=true ! mux."

nightVision="v4l2src device=/dev/v4l/by-id/usb-e-con_systems_See3CAM_CU27_3B1519112B010900-video-index0 ! jpegdec ! video/x-raw, height=1080 ! $encoder"

SecondaryTopCam="v4l2src do-timestamp=true device=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0 ! video/x-raw, height=480, width=640, framerate=30/1 ! $encoder"

ArmScienceCam="v4l2src device=/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0 ! video/x-raw, height=480, width=640, framerate=30/1 ! $encoder" # v4l2src stuff here

Help()
{
    # Display Help
    echo "Start streaming cameras using gstreamer. Need VLC to display them."
    echo
    echo "Syntax: scriptTemplate [-n|-t|-a]"
    echo "options:"
    echo "n     Enable night vision camera."
    echo "t     Enable secondary top camera."
    echo "a     Enable arm/science camera."
    echo
}


#
# Define argument variables
#
ENABLE_NIGHTVISION_CAM=0
ENABLE_SECONDARY_TOP_CAM=0
ENABLE_ARM_SCIENCE_CAM=0

#
# Parse arguments
#
while (( $# > 0 )) ; do
    case $1 in 
        -n | --night-vision)
            ENABLE_NIGHTVISION_CAM=1
            ;;
        -t | --top)
            ENABLE_SECONDARY_TOP_CAM=1
            ;;
        -a | --arm)
            ENABLE_ARM_SCIENCE_CAM=1
            ;;
        -h | --help)
            Help
            exit 0
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
echo "ENABLE_ARM_SCIENCE_CAM: $ENABLE_ARM_SCIENCE_CAM"

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

if [ "$ENABLE_ARM_SCIENCE_CAM" == "1" ]; then
    GST_CAM_SETTINGS="$GST_CAM_SETTINGS $ArmScienceCam"
    echo "Adding arm science camera"
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


