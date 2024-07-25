import cv2
import threading
from time import sleep

"""
Captures video from a camera and always gives the latest frame.
"""
class VideoCapture:
    def __init__(self, cam_index: int, xRes: int, yRes, filename_prefix: str, output_directory: str):
        self.cam_index = cam_index
        self.xRes = xRes
        self.yRes = yRes
        self.filename_prefix = filename_prefix
        self.output_directory = output_directory
        self.images_captured = 0

        if not self.output_directory.endswith("/"):
            self.output_directory += "/"
        
        try:
            self.cap = cv2.VideoCapture(cam_index)
        except Exception as e:
            raise ValueError(f"Could not find the video index for the camera. Error: {e}")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.xRes)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.yRes)

        self.lock = threading.Lock()
        self.t = threading.Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

    
    def close(self):
        self.cap.release()
    
    def save_images(self, num_images: int, time_between_images: float):
        for i in range(0, num_images):
            sleep(time_between_images)
            self._save_image()

    def _save_image(self):
        self.images_captured += 1
        cv2.imwrite(f"{self.output_directory}{self.filename_prefix}-Image{self.images_captured}.png", self._read())

    def _reader(self):
        """
        Grab frames as soon as they are available
        """
        while True:
            with self.lock:
                ret = self.cap.grab()
            if not ret:
                break
            sleep(0.01)

    def _read(self):
        """
        Retrieve the latest frame.
        """
        with self.lock:
            _, frame = self.cap.retrieve()

        return frame