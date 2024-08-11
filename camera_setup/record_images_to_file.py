import cv2, os, re, argparse
from time import sleep

def _find_video_index(v4l_byid_name: str):
    if v4l_byid_name is None or len(v4l_byid_name) == 0:
        raise ValueError("Must provide a v4l_byid_name name to find the video index")
    
    v4l_byid_name = "/dev/v4l/by-id/" + v4l_byid_name

    if not os.path.exists(v4l_byid_name):
        raise ValueError("Path doesn't exist when following the v4l_byid_name")
    
    device_path = os.path.realpath(v4l_byid_name)
    device_re = re.compile("\/dev\/video(\d+)")
    info = device_re.match(device_path)
    if not info:
        raise ValueError("Could not find the video index in the file pointed to by v4l_byid_name")
    
    return int(info.group(1))

def main():

    # For non v4l2 loopback and direct control
    v4l_byid_name = "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0"
    video_index = _find_video_index(v4l_byid_name)

    # If v4l2 is active it will pipe the camera to /dev/video16
    video_index = 16

    parser = argparse.ArgumentParser(prog="Record images to file")
    
    parser.add_argument('-n', '--name')

    args = parser.parse_args()

    if args.name is None:
        raise ValueError("Must provide a name for the directory")

    dir_name = os.path.join("images", args.name)

    if not os.path.exists(dir_name):
        os.makedirs(dir_name)
        print(f"Creating directory named {dir_name}")
    else:
        print(f"Directory {dir_name} already exists. Moving on")

  
    cap = cv2.VideoCapture(_find_video_index(v4l_byid_name))
    if cap is None or not cap.isOpened():
        raise ValueError("Could not open the camera as " + str(v4l_byid_name))

    i = 1
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imwrite(os.path.join(dir_name, f'ParanormalIndividualImage{i}.jpg'), frame)
            i+=1

            if False:
                cv2.imshow('frame', frame)
                cv2.waitKey(0)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            print("Recording " + str(i))
            sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
