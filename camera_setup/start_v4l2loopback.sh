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
# - /dev/video14 -> IRCamera
# - /dev/video15 -> PlaceHolder1
# - /dev/video16 -> PlaceHolder2

sudo modprobe v4l2loopback devices=7 video_nr=10,11,12,13,14,15,16 exclusive_caps=1,1,1,1,1,1,1 card_label="ZED2i,ExpensiveAssLowLightCamera,ScienceCamera,ArmCamera,IRCamera,PlaceHolder1,PlaceHolder2"
