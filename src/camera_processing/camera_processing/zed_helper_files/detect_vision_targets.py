import cv2
import pyzed.sl as sl
import numpy as np
from enum import Enum
from typing import List

# from ZedNode import STRING_TO_RESOLUTION
from ..HSVImageExplore.image_colour_processing.colour_processing import ColourProcessing

from .map_targets_between_cams import PitchYaw, Point, CameraUtil



# ELP 640x480
# elp_width = 640
# elp_height = 480
# elp_hfov = 49.29*2
# elp_vfov = 37.91*2

# ELP 1280x720
elp_width = 1280
elp_height = 720
elp_hfov = 62.91*2
elp_vfov = 37.91*2

# ELP 1920x1080
# elp_width = 1920
# elp_height = 1080
# elp_hfov = 62.91*2
# elp_vfov = 37.91*2

ZED_CAM_NAME = "ZED"
IR_CAM_NAME = "IRCam"

class CameraType(Enum):
    ZED = CameraUtil(ZED_CAM_NAME, 2208, 1242, 110, 70)
    ERIK_ELP = CameraUtil(IR_CAM_NAME, elp_width, elp_height, elp_hfov, elp_vfov)

class DetectVisionTargets:
    def __init__(self, blue_led: ColourProcessing, red_led: ColourProcessing, ir_led: ColourProcessing):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params =  cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.blue_led_processing = blue_led
        self.red_led_processing = red_led
        self.ir_led_processing = ir_led

    def bounding_box(points: np.array):
        """
        Find min/max from an N-collection of coordinate pairs, shape = (N, 2), using 
        numpy's min/max along the collection-axis 
        """
        xMin = min(point[0] for point in points)
        yMin = min(point[1] for point in points)
        xMax = max(point[0] for point in points)
        yMax = max(point[1] for point in points)

        box = np.zeros((4, 2))

        box[0][0] = xMin
        box[0][1] = yMin

        box[1][0] = xMax
        box[1][1] = yMin

        box[2][0] = xMax
        box[2][1] = yMax

        box[3][0] = xMin
        box[3][1] = yMax

        return box

    def create_aruco_unique_label(camera_mapping: CameraType, marker_id: int) -> str:
        return f"{camera_mapping.value.name}-ArucoID{marker_id}"

    def is_zed_marker(unique_label: str) -> bool:
        return f"{ZED_CAM_NAME}-ArucoID" in unique_label
    
    def is_ir_cam_marker(unique_label: str) -> bool:
        return f"{IR_CAM_NAME}-ArucoID" in unique_label
    
    def get_marker_id_from_label(unique_label: str) -> int:
        try:
            return int(unique_label[unique_label.index("-ArucoID")+len("-ArucoID"):])
        except Exception as e:
            return -1

    def detectArucoMarkers(self, img, cameraMapping: CameraType) -> List[sl.CustomBoxObjectData]:
        try:
            corners, ids, rejected = self.aruco_detector.detectMarkers(img)
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            detections = []

            if len(corners) == 0:
                print("No aruco markers found")

            elif len(corners) != len(ids):
                print("Mismatched id and corners arrays from the aruco detector")

            else:
                for i in range(0, len(corners)):
                    markerCorners = corners[i]
                    markerID: int = ids[i][0]

                    reshapedCorners = DetectVisionTargets.bounding_box(markerCorners.reshape((4, 2)))

                    if (cameraMapping != CameraType.ZED):
                        for points in reshapedCorners:
                            # print(f"    REMAPPING: {points}")
                            pitchYaw = cameraMapping.value.pitchYawFromXY(Point(points[0], points[1]))
                            xy = CameraType.ZED.xyFromPitchYaw(pitchYaw)
                            points[0] = max(0, xy.x) # Ensure values are not negative
                            points[1] = max(0, xy.y)
                            # print(f"           TO: {points}")
                            # print(f"     PitchYaw: {pitchYaw}")

                    # Creating ingestable objects for the ZED SDK
                    obj = sl.CustomBoxObjectData()
                    obj.unique_object_id = DetectVisionTargets.create_aruco_unique_label(cameraMapping, markerID)
                    obj.bounding_box_2d = reshapedCorners # Converts to unsigned int so values MUST be positive or it fails
                    obj.label = int(markerID)
                    obj.probability = 0.99
                    obj.is_grounded = False
                    detections.append(obj)

            return detections

        except Exception as e:
            print(f"Encounted exception trying to detect aruco markers: {e}")
            return detections


    def detectZEDBlueLEDs(self, zed_img) -> List[sl.CustomBoxObjectData]:
        detections = []

        # Blue LEDS
        mask = self.blue_led_processing.process_mask(zed_img)
        bounding_boxes = self.blue_led_processing.process_contours(mask, zed_img)

        for i in range(0, len(bounding_boxes)):
            obj = sl.CustomBoxObjectData()
            obj.unique_object_id = f"BlueLED-{i}"
            obj.bounding_box_2d = bounding_boxes[i]
            obj.label = 3
            obj.probability = 0.99
            obj.is_grounded = False
            detections.append(obj)

        return detections
    
    def detectZEDRedLEDs(self, zed_img) -> List[sl.CustomBoxObjectData]:
        detections = []

        # Red LEDS
        mask = self.red_led_processing.process_mask(zed_img)
        bounding_boxes = self.red_led_processing.process_contours(mask, zed_img)

        for i in range(0, len(bounding_boxes)):
            obj = sl.CustomBoxObjectData()
            obj.unique_object_id = f"RedLED-{i}"
            obj.bounding_box_2d = bounding_boxes[i]
            obj.label = 4
            obj.probability = 0.99
            obj.is_grounded = False
            detections.append(obj)

        return detections

    def detectIRLEDS(self, ir_img, cameraMapping: CameraType) -> List[sl.CustomBoxObjectData]:
        detections = []
        return detections

    def draw_object_detection(img, object_data: sl.ObjectData):
        (topLeft, topRight, bottomRight, bottomLeft) = object_data.bounding_box_2d

        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # Draw the bounding box
        cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

        # Compute and draw the center (x, y)-coordinates
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)

        # TODO: Change to a better label for LEDs and Arucos
        text = str(object_data.unique_object_id)
        cv2.putText(img, text, (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)