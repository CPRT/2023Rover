import cv2
from . import cv2_helper
from numpy import ndarray

class MaskStep:
    def __init__(self, window_name: str):
        self._window_name = window_name
        self._is_display_active = False
        self._display_scaling = 1.0

    def process(self, original_img: ndarray, process_img: ndarray) -> ndarray:
        """
        Process an image and return the newly processed image.

        Parameters:
            original_img (ndarray): The original image with no processing
            process_img (ndarray): The processed image returned from the previous step

        Returns:
            ndarray: The newly processed image

        Raises: 
            NotImplementedError - This method must be overriden by a subclass
        """
        raise NotImplementedError

    def _create_display(self) -> str:
        """
        Create the cv2 window for this step

        Returns:
            str: The name of the new cv2 window

        Raises: 
            NotImplementedError - This method must be overriden by a subclass
        """
        raise NotImplementedError
    
    def _destory_display(self):
        """
        Destroy the cv2 windows for this step.

        Raises: 
            NotImplementedError - This method must be overriden by a subclass
        """
        raise NotImplementedError
    
    def imshow_scaled(self, window_name: str, image: ndarray):
        """
        Display an image on a window with the given name.
        Creates the window if it isn't already created with create_window.

        Parameters:
            window_name (str): The name for the window to display the image to
            image (ndarray): The image to display
            image_scaling (float): A float to scale the image by
        """
        cv2_helper.imshow_scaled(window_name, image, self._display_scaling)
        
    def start_display(self) -> str:
        """
        Create the cv2 window for this step.

        Returns:
            str: The name of the new cv2 window
        """
        self._is_display_active = True
        self._create_display()

    def stop_display(self) -> str:
        """
        Stop the cv2 window for this step.
        """
        self._is_display_active = False
        self._destory_display()

    def set_display_scaling(self, display_scaling: float):
        """
        Set the image scaling for this mask step.
        """
        self._display_scaling = display_scaling