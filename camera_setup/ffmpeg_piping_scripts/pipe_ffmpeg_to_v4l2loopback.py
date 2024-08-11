from ffmpeg.ffmpeg import FFmpeg
from ffmpeg.progress import Progress
import os, sys, argparse, re
from typing import Tuple

CAMERA_PROFILE_NAMES = ['ZED', 'IR_ELP', 'PS3_EYE', 'ExpensiveAssLowLightCamera', 'PanoramaCamera']

"""
Steps to add a new camera
    1. Add the name of the new camera profile to CAMERA_PROFILE_NAMES above
    2. Create a function to find the camera's video index and the desired output video index
    3. Add the same camera profile name to the if elif statement in the get_input_output_indexes function
"""

def get_input_output_indexes(camera_profile: str) -> Tuple[int, int]:
    if camera_profile == 'ZED'.lower():
        return ZED()
    elif camera_profile == 'ExpensiveAssLowLightCamera'.lower():
        return ExpensiveAssLowLightCamera()    
    elif camera_profile == 'IR_ELP'.lower():
        return IR_ELP()
    elif camera_profile == 'PS3_EYE'.lower():
        return PS3_EYE()
    elif camera_profile == 'PanoramaCamera'.lower():
        return PanoramaCamera()
    else:
        exit_with_error(os.EX_USAGE, f"Unable to determine camera_profile in get_input_output_indexes")

def ZED() -> Tuple[int, int]:
    output_index = 10
    camera_index = find_video_index_by_id("usb-Technologies__Inc._ZED_2i_OV0001-video-index0")
    return camera_index, output_index

def ExpensiveAssLowLightCamera() -> Tuple[int, int]:
    output_index = 11
    camera_index = find_video_index_by_id("usb-e-con_systems_See3CAM_CU27_3B1519112B010900-video-index0")
    return camera_index, output_index

def IR_ELP() -> Tuple[int, int]:
    output_index = 14
    camera_index = find_video_index_by_id("usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0")
    return camera_index, output_index

def PS3_EYE() -> Tuple[int, int]:
    output_index = 15
    camera_index = find_video_index_by_id("usb-OmniVision_Technologies__Inc._USB_Camera-B4.09.24.1-video-index0")
    return camera_index, output_index

def PanoramaCamera()-> Tuple[int, int]:
    output_index = 16
    camera_index = find_video_index_by_id("usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0")
    return camera_index, output_index

def find_video_index_by_id(v4l_byid_name: str) -> int:
    """
    Searches for the given v4l_byid_name in the /dev/v4l/by-id/ directory.
    Follows the symlink to get a device index

    Parameters:
        v4l_byid_name(str): A string containing a camera by-id name

    Returns:
        int: The video index of the camera
    """
    if v4l_byid_name is None or len(v4l_byid_name) == 0:
        exit_with_error(os.EX_SOFTWARE, "Tried to find camera device index with empty v4l_byid_name")
    
    v4l_byid_name = "/dev/v4l/by-id/" + v4l_byid_name

    if not os.path.exists(v4l_byid_name):
        exit_with_error(os.EX_SOFTWARE, f"Tried to find camera device index but path followed by v4l_byid_name is not valid. v4l_byid_name: {v4l_byid_name}")
    
    device_path = os.path.realpath(v4l_byid_name)
    device_re = re.compile("\/dev\/video(\d+)")
    info = device_re.match(device_path)
    if not info:
        exit_with_error(os.EX_SOFTWARE, f"Could not find the video index in the file pointed to by v4l_byid_name. v4l_byid_name: {v4l_byid_name}")

    return int(info.group(1))




def main():
    try:
        parser = argparse.ArgumentParser(
            prog="Colour Processing Images",
            description="Processes images following multiple mask steps with tuning features")
        
        parser.add_argument('--camera', help=f"Camera by name. Options are {repr(CAMERA_PROFILE_NAMES)}")
        args = parser.parse_args()

        if args.camera is None:
            exit_with_error(os.EX_USAGE, f"Must provide a camera profile name")

        try:
            selected_profile = str(args.camera).lower()
        except ValueError as e:
            if selected_profile is not None:
                exit_with_error(os.EX_USAGE, f"Must proivde the camera profile with --camera (provided: {selected_profile}, with type: {type(selected_profile)}")
            else:
                exit_with_error(os.EX_USAGE, f"Must provide the camera profile with --camera")

        camera_options_lower = []
        for cam in CAMERA_PROFILE_NAMES:
            camera_options_lower.append(cam.lower())

        if selected_profile not in camera_options_lower:
            exit_with_error(os.EX_USAGE, f"Invalid camera profile (provided: {selected_profile}). Options are {repr(CAMERA_PROFILE_NAMES)}")

    except Exception as e:
        exit_with_error(os.EX_USAGE, f"Failed to parse args with exception: {e}")

    cam_index, output_index = get_input_output_indexes(selected_profile)
    printS(f"Camera Device: /dev/video{cam_index}, Output Device: /dev/video{output_index}")

    # try:
    #     if not os.path.isfile(f"/dev/video{cam_index}"):
    #         exit_with_error(os.EX_SOFTWARE, f"Camera device file does not exist: /dev/video{cam_index}")

    #     if not os.path.isfile(f"/dev/video{output_index}"):
    #         exit_with_error(os.EX_SOFTWARE, f"Output device file does not exist: /dev/video{output_index}")

    # except Exception as e:
    #     exit_with_error(os.EX_SOFTWARE, f"Failed to check if camera and output device files exist with exception: {e}")


# 640 by 480 at a FPS of 30
    try:
        # Same as command: ffmpeg -f v4l2 -i /dev/video0 -f v4l2 /dev/video10
        ffmpeg = ffmpeg = (
            FFmpeg()
            .option("y")
            .input(
                '/dev/video' + str(cam_index),
                f='rawvideo',
                framerate=30,
                video_size='640x480'
                # pixel_format='yuv420p'
            )
            .output(
                '/dev/video' + str(output_index), 
                f='rawvideo',
            )
        )
    except Exception as e:
        exit_with_error(os.EX_SOFTWARE, f"Failed to create FFmpeg object with exception: {e}")

    @ffmpeg.on("progress")
    def time_to_terminate(progress: Progress):
        printS(f"{selected_profile} - {get_progress_as_string(progress)}")

    try:
        result = ffmpeg.execute()
    except KeyboardInterrupt as e:
        ffmpeg.terminate()
        printS(f"[PythonPrint]: KeyboardInterrupt: Terminated ffmpeg instance")
    except Exception as e:
        exit_with_error(os.EX_SOFTWARE, f"Exception from ffmpeg: {e}")


def get_progress_as_string(progress: Progress):
    return f"frame: {progress.frame}, fps: {progress.fps}, size: {progress.size}, " + \
            f"time: {progress.time}, bitrate: {progress.bitrate}, speed: {progress.speed}"

def exit_with_error(error_code: int, msg: str):
    printS(msg)
    printS("Exiting with error code: " + str(error_code))
    sys.exit(error_code)

def printS(string: str):
    print("[PythonPrint]: " + str(string))

if __name__ == '__main__':
    main()