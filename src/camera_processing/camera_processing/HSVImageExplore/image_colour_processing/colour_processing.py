from typing import Tuple, Callable, List, Union
from __future__ import annotations
from .mask_step import MaskStep

import cv2
from . import cv2_helper
from numpy import ndarray
import time
import numpy as np

class ColourProcessing:
    def __init__(self, image_scaling: float, display_scaling: float, mask_steps: Tuple[MaskStep]):
        """
        """
        self._in_tuning_mode: bool
        self._mask_steps: Tuple[MaskStep] = mask_steps
        self._image_scaling: float = image_scaling
        self._display_scaling: float = display_scaling
    
        if len(mask_steps) == 0:
            raise IndexError("Must give at least one MaskStep to ColourProcessing")
        
        for step in mask_steps:
            step.set_display_scaling(display_scaling)

    @classmethod    
    def from_string(python_eval: str) -> Union[ColourProcessing, str]:
        """
        Create a ColourProcessing object from a string that can be evaluated to a ColourProcessing object.
        The intention is for HSVImageExplore to print how strings that can be eval'd into ColourProcessing objects.

        Parameters:
            - python_eval (str): The string to eval into a ColourProcessing object
        
        Returns:
            - Union[ColourProcessing, str]: The ColourProcessing object or an error message
        """
        try:
            obj = eval(python_eval)
        except Exception as e:
            return f"Error creating ColourProcessing object. Error: {e}"
        
        if not isinstance(obj, ColourProcessing):
            return f"Error creating ColourProcessing object. Error: {obj} is not a ColourProcessing object."
        
        return obj
    
    def process_mask(self, image: ndarray) -> ndarray:
        """
        Take an image and create a mask by running it through all the MaskSteps of this object.

        Paramters:
            image (ndarray): The image to process

        Returns:
            ndarray: The mask for the image
        """
        start = time.time()
        mask_timings = []
        processed_image = image
        for mask_step in self._mask_steps:
            step_start = time.time()
            processed_image = mask_step.process(image, processed_image)
            mask_timings.append(time.time() - step_start)

        print(f"Process time: {time.time() - start:.4f}. Steps: {np.around(mask_timings, 4)}")
        return processed_image
    
    def process_contours(self, mask: ndarray, draw_contours_img: ndarray = None) -> list:
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)   # cv2.CHAIN_APPROX_NONE   cv2.CHAIN_APPROX_SIMPLE

        if draw_contours_img is not None:
            cv2.drawContours(draw_contours_img, contours, contourIdx=-1, color=(0, 255, 0), thickness=3, hierarchy=hierarchy)

        detections = []

        for i in range(0, len(contours)):
            c = contours[i]
            h = hierarchy[0][i]
            if h[2] != -1:
                child_area = cv2.contourArea(contours[h[2]])
                percent = int(100 * child_area / cv2.contourArea(c))
            else:
                child_area = 0
                percent = 0

            # If it doesn't have a child contour, its the outermost contour
            if h[2] != -1:
                detections.append(ColourProcessing._contour_to_bounding_box(c))

            print(f"Contour Stuffs: Area: {cv2.contourArea(c)}. Child Area: {child_area}. Percent {percent}")
        return detections
    
    def _contour_to_bounding_box(contour) -> list:
        xywh = cv2.boundingRect(contour)
        return ColourProcessing._xywh2abcd(xywh)
 
    def _xywh2abcd(xywh):
        output = np.zeros((4, 2))

        x_min = (xywh[0]) # * im_shape[1]
        x_max = (xywh[0] + xywh[2]) # * im_shape[1]
        y_min = (xywh[1]) # * im_shape[0]
        y_max = (xywh[1] + xywh[3]) # * im_shape[0]

        # A ------ B
        # | Object |
        # D ------ C

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_max
        output[2][1] = y_max

        output[3][0] = x_min
        output[3][1] = y_max
        return output

    def single_image_mask(self, filename: str):
        image: ndarray = cv2.imread(filename)
        print(f"Image size: {image.shape}")
        image = cv2.resize(image, None, fx=self._image_scaling, fy=self._image_scaling, interpolation = cv2.INTER_AREA)
        print(f"Image size scaled: {image.shape}")

        for i in range(0, 10):
            self.process_contours(self.process_mask(image))

    def mask_step_tuning(self, image: ndarray, starting_step: int = 0) -> bool:
        image = cv2.resize(image, None, fx=self._image_scaling, fy=self._image_scaling, interpolation = cv2.INTER_AREA)

        mask_step_index: int = starting_step
        self._mask_steps[mask_step_index].start_display()
        
        while True:
            self.process_mask(image)            
            
            key_pressed = cv2.waitKey(5)
            new_index = None

            if key_pressed == cv2_helper.WaitKeyStroke.ESC or key_pressed == ord('s') or key_pressed == ord('d'):
                self._mask_steps[mask_step_index].stop_display()
                print(f"The Mask Steps are below:\n(")
                for step in self._mask_steps:
                    print(f"{repr(step)}, ")
                print(")")
                return key_pressed, mask_step_index
            
            elif key_pressed == ord('w'):
                new_index = max(mask_step_index - 1, 0)
            elif key_pressed == ord('e'):
                new_index = min(mask_step_index + 1, len(self._mask_steps) - 1)

            else:
                for i in range(1, len(self._mask_steps)+1):
                    if key_pressed == ord(str(i)):
                        new_index = i - 1
                        break
                        
            if new_index is not None and new_index != mask_step_index:
                print(f"Stopping {mask_step_index} and starting {new_index}")
                self._mask_steps[mask_step_index].stop_display()
                self._mask_steps[new_index].start_display()
                mask_step_index = new_index

    def single_image_mask_tuning(self, filename: str):
        image: ndarray = cv2.imread(filename)
        print(f"Image size: {image.shape}")
        
        mask_step_index: int = 0
        self._mask_steps[mask_step_index].start_display()
        
        self.mask_step_tuning(image)
        cv2.destroyAllWindows()

    def multi_image_mask_tuning(self, image_callback: Callable[..., ndarray]):
        images: List[ndarray] = [image_callback()]
        image_index: int = 0
        mask_step: int = 0
        while True:
            key_pressed, mask_step = self.mask_step_tuning(images[image_index], mask_step)

            if key_pressed == cv2_helper.WaitKeyStroke.ESC:
                break
            elif key_pressed == ord('s'):
                image_index = max(0, image_index - 1)
            elif key_pressed == ord('d'):
                image_index += 1
                if image_index >= len(images):
                    images.append(image_callback())


        cv2.destroyAllWindows()
