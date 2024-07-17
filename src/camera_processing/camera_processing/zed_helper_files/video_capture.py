import cv2
import threading

from .detect_vision_targets import CameraType

"""
Captures video from a camera and always gives the latest frame.
"""
class VideoCapture:
    def __init__(self, name, cam: CameraType = None):
        self.xRes = cam.value.xRes
        self.yRes = cam.value.yRes

        self.cap = cv2.VideoCapture(name)
        self.cam_type = cam
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.xRes)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.yRes)

        self.lock = threading.Lock()
        self.t = threading.Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

    def _reader(self):
        """
        Grab frames as soon as they are available
        """
        while True:
            with self.lock:
                ret = self.cap.grab()
            if not ret:
                break

    def read(self):
        """
        Retrieve the latest frame.
        """
        with self.lock:
            _, frame = self.cap.retrieve()
        return frame