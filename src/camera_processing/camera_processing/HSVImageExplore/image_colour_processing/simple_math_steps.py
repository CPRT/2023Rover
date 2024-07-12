import cv2
from numpy import ndarray
import numpy as np
from typing import Optional, Tuple, List, Set
from copy import deepcopy

try:
    # Import for CLI usage
    import cv2_helper
    from process_steps import MaskToMathStep, MathStep
    print(f"{__name__} is being used as a script")

except Exception as e:
    # Import for ROS usage
    from . import cv2_helper
    from .process_steps import MaskToMathStep, MathStep

# 
class FindContours(MaskToMathStep):
    
    def __init__(self, window_name: str, mode: int = cv2.RETR_CCOMP, method: int = cv2.CHAIN_APPROX_SIMPLE):
        """
        
        Example: FindContours("Find Contours", cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        """
        super().__init__(window_name)

        self._mode = mode
        self._method = method

    def __repr__(self) -> str:
        return f"FindContours({repr(self._window_name)})"
    
    def process(self, original_image: ndarray, mask: ndarray) -> Tuple[list, list]:
        """
        Process contours to remove 

        Parameters:
            original_img (ndarray): The original image with no processing
            mask (ndarray): The mask produced from the MaskSteps

        Returns:
            Tuple[list, list]: The contours and hierarchy of the mask from cv2.findContours
        """
        contours, hierarchy = cv2.findContours(mask, self._mode, self._method)

        if self._is_display_active:

            mask_img = np.zeros(original_image.shape, original_image.dtype)
            mask_img[:, :] = (255, 255, 255)
            mask_image_contours = cv2.bitwise_and(mask_img, mask_img, mask=mask) # Add back in a colour channel to display it with original_img (must be the same shape for np.hstack)

            cv2.drawContours(mask_image_contours, contours, -1, (0, 255, 0), 2)

            images_stacked_horizontally = np.hstack([original_image, mask_image_contours])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return contours, hierarchy
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)


class FilterByContourArea(MathStep):
    REJECT_BELOW_NAME = "Reject Below"
    FAR_CLOSE_DIVIDER_NAME = "Far Close Divider"
    REJECT_ABOVE_NAME = "Reject Above"

    def __init__(self, window_name: str, reject_below: int, far_close_divider: int, reject_above: int, area_scaling: float = 1.0):
        """
        Remove contours below an area threshold from reject_below, tag contours below far_close_divider as far, 
        tags contours above far_close_divider as close, and remove contours above reject_above.
        """
        super().__init__(window_name)

        self._reject_below = int(reject_below * area_scaling)
        self._far_close_divider = int(far_close_divider * area_scaling)
        self._reject_above = int(reject_above * area_scaling)

        self._area_scaling = area_scaling
        

    def __repr__(self) -> str:
        return f"FilterByContourArea({repr(self._window_name)}, reject_below={self._reject_below}, " + \
                    f"far_close_divider={self._far_close_divider}, reject_above={self._reject_above})"
    
    def process(self, original_image: ndarray, mask: ndarray, contours: list, hierarchy: list, tags: List[Set[str]]) -> Tuple[list, list, List[Set[str]]]:
        """
        Process contours to remove bad contours or tag various contours

        Parameters:
            original_img (ndarray): The original image with no processing
            mask (ndarray): The mask produced from the MaskSteps
            contours (list): The original image with no processing
            hierarchy (list): The processed image returned from the previous step
            tags (List[Set[str]]): The tags for each contour

        Returns:
            ndarray: The newly processed image

        Raises: 
            NotImplementedError - This method must be overriden by a subclass
        """
        new_contours = []
        new_hierarchy = []
        new_tags = []

        removed_contours = []

        print(f"Contours: {len(contours)}")
        print(f"Hierarchy: {len(hierarchy)}")
        print(f"Tags: {len(tags)}")


        for i in range(0, len(contours)):
            area = cv2.contourArea(contours[i])

            if area < self._reject_below:
                removed_contours.append(contours[i])
                
            elif area < self._far_close_divider:
                tags[i].add("far")
                new_contours.append(contours[i])
                new_hierarchy.append(hierarchy[i])
                new_tags.append(tags[i])

            elif area < self._reject_above:
                tags[i].add("close")
                new_contours.append(contours[i])
                new_hierarchy.append(hierarchy[i])
                new_tags.append(tags[i])

            else:
                removed_contours.append(contours[i])

        print(f"Removed {len(removed_contours)} contours")
        print(f"New Contours: {len(new_contours)}")
        print(f"New Hierarchy: {len(new_hierarchy)}")
        print(f"New Tags: {new_tags}")

        if self._is_display_active:
            self._reject_below = cv2.getTrackbarPos(FilterByContourArea.REJECT_BELOW_NAME, self._window_name)
            self._far_close_divider = cv2.getTrackbarPos(FilterByContourArea.FAR_CLOSE_DIVIDER_NAME, self._window_name)
            self._reject_above = cv2.getTrackbarPos(FilterByContourArea.REJECT_ABOVE_NAME, self._window_name)

            mask_img = np.zeros(original_image.shape, original_image.dtype)
            mask_img[:, :] = (255, 255, 255)
            mask_image_contours = cv2.bitwise_and(mask_img, mask_img, mask=mask) # Add back in a colour channel to display it with original_img (must be the same shape for np.hstack)

            cv2.drawContours(mask_image_contours, removed_contours, -1, (0, 0, 255), 2)

            for i in range(0, len(new_contours)):
                if "close" in new_tags[i]:
                    x, y, w, h = cv2.boundingRect(new_contours[i])
                    cv2.drawContours(mask_image_contours, new_contours, i, (255,25,12), 2)
                    cv2.putText(mask_image_contours, f"close-{int(cv2.contourArea(new_contours[i]))}", (x, max(y-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,25,12), 2)
                elif "far" in new_tags[i]:
                    x, y, w, h = cv2.boundingRect(new_contours[i])
                    cv2.drawContours(mask_image_contours, new_contours, i, (36,255,12), 2)
                    cv2.putText(mask_image_contours, f"far-{int(cv2.contourArea(new_contours[i]))}", (x, max(y-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (36,255,12), 2)
                else:
                    print(f"EDGE CASE. i: {i}, tags[i]: {tags[i]}")
            for i in range(0, len(removed_contours)):
                x, y, w, h = cv2.boundingRect(removed_contours[i])
                cv2.putText(mask_image_contours, f"reject-{int(cv2.contourArea(removed_contours[i]))}", (x, max(y-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            images_stacked_horizontally = np.hstack([original_image, mask_image_contours])
            self.imshow_scaled(self._window_name, images_stacked_horizontally)

        return contours, hierarchy, tags
  
    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window
        """
        cv2.namedWindow(self._window_name)
        cv2.createTrackbar(FilterByContourArea.REJECT_BELOW_NAME, self._window_name, self._reject_below, 1000, cv2_helper.do_nothing)
        cv2.createTrackbar(FilterByContourArea.FAR_CLOSE_DIVIDER_NAME, self._window_name, self._far_close_divider, 6000, cv2_helper.do_nothing)
        cv2.createTrackbar(FilterByContourArea.REJECT_ABOVE_NAME, self._window_name, self._reject_above, 20000, cv2_helper.do_nothing)

    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.
        """
        cv2.destroyWindow(self._window_name)