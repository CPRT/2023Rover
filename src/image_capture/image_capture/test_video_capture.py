from video_capture import VideoCapture
from time import sleep

def main():
    vid_cap = VideoCapture(0, 640, 480, "test_name", "/home/erik/GitHub/2023Rover/src/image_capture/image_capture/test_dir")

    sleep(5)

    vid_cap.save_images(5, 2)


    vid_cap.close()

if __name__ == "__main__":
    main()