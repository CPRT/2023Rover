
start="gst-launch-1.0"

end="mpegtsmux name=mux alignment=7 ! srtsink uri=srt://:9000 latency=200 sync=true"

encoder="nvvidconv ! nvv4l2h265enc bitrate=5000000 control-rate=0 maxperf-enable=true insert-sps-pps=true ! mux."

nightVision="v4l2src device=/dev/v4l/by-id/usb-e-con_systems_See3CAM_CU27_3B1519112B010900-video-index0 ! $encoder"

SecondaryTopCam="v4l2src device=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0 ! video/x-raw, height=480, width=640, framerate=30/1 ! $encoder"

cmd="${start} ${nightVision} ${SecondaryTopCam} ${end}"

eval ${cmd}
