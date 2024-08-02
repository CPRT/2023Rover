#!/bin/bash 





# Use  > /dev/null to tell ffmpeg to not look for input from terminal

ffmpeg -f v4l2 -i /dev/video0 -f v4l2 /dev/video2 > /dev/null 2>&1 &