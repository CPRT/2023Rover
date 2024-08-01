from numpy import ndarray
import numpy
from typing import Tuple
from copy import deepcopy

try:
    # Import for CLI usage
    import cv2_helper
    print(f"{__name__} is being used as a script")

except Exception as e:
    # Import for ROS usage
    from . import cv2_helper

class HSV:
    def __init__(self, hue: int, sat: int, val: int):
        self.hue = hue
        self.saturation = sat
        self.value = val

    def __repr__(self) -> str:
        return f"HSV({self.hue}, {self.saturation}, {self.value})"
    
    def asTuple(self) -> Tuple[int]:
        return (self.hue, self.saturation, self.value)

class HSVRange:
    def __init__(self, low: HSV, high: HSV):
        self.low = low
        self.high = high

    def __repr__(self) -> str:
        return f"HSVRange({repr(self.low)}, {repr(self.high)})"
    
    def apply_range(self, hsv_image: ndarray, invert_hue: bool = False):
        if not invert_hue:
            return cv2_helper.inRange(hsv_image, self.low.asTuple(), self.high.asTuple())
        else:
            low = deepcopy(self.low)
            high = deepcopy(self.high)
            low.hue = 0
            high.hue = self.low.hue
            low_mask = cv2_helper.inRange(hsv_image, low.asTuple(), high.asTuple())

            low = deepcopy(self.low)
            high = deepcopy(self.high)
            low.hue = self.high.hue
            high.hue = cv2_helper.HUE_MAX
            high_mask = cv2_helper.inRange(hsv_image, low.asTuple(), high.asTuple())

            return low_mask | high_mask