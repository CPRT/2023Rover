#!/bin/bash 

# Creates virtual cameras.
# Use ffmpeg to pipe video feeds into these virtual cameras
# The virtual camera can then allow multiple programs to read it at the same time
# 
# Cameras (virtual camera device -> given card_label):
# - /dev/video10 -> ZED2i
# - /dev/video11 -> ExpensiveAssLowLightCamera
# - /dev/video12 -> ScienceCamera
# - /dev/video13 -> ArmCamera
# - /dev/video14 -> IR_ELP
# - /dev/video15 -> PS3_EYE
# - /dev/video16 -> PanoramaCamera

# Unload v4l2loopback
sudo modprobe -r v4l2loopback

# Load v4l2loop
sudo modprobe v4l2loopback devices=7 video_nr=10,11,12,13,14,15,16 exclusive_caps=1,1,1,1,1,1,1 card_label="ZED2i,ExpensiveAssLowLightCamera,ScienceCamera,ArmCamera,IR_ELP,PS3_EYE,PlaceHolder2"
