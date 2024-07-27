import pyzed.sl as sl
import rclpy
from sensor_msgs.msg import Image
from enum import Enum

class ImageEncoding(Enum):
    """
    Enum for image encodings.
    """
    TYPE_32FC1 = "32FC1"
    TYPE_32FC2 = "32FC2"
    TYPE_32FC3 = "32FC3"
    TYPE_32FC4 = "32FC4"
    MONO8 = "mono8"
    TYPE_8UC2 = "8UC2"
    BGR8 = "bgr8"
    BGRA8 = "bgra8"


def imageToROSMsg(img: sl.Mat, frame_id: str, timestamp) -> Image:
    """
    Convert a ZED image to a ROS message.
    """
    imgMessage = Image()

    imgMessage.header.stamp = timestamp
    imgMessage.header.frame_id = frame_id
    imgMessage.height = img.get_height()
    imgMessage.width = img.get_width()

    imgMessage.is_bigendian = False

    imgMessage.step = img.get_step_bytes()

    dataType: sl.MAT_TYPE = img.get_data_type()

    imgMessage.data = img.get_data(memory_type=sl.MEM.CPU, deep_copy=True).tobytes()

    if dataType == sl.MAT_TYPE.F32_C1: # float 1 channel
        imgMessage.encoding = ImageEncoding.TYPE_32FC1.value

    elif dataType == sl.MAT_TYPE.F32_C2: # float 2 channels
        imgMessage.encoding = ImageEncoding.TYPE_32FC2.value

    elif dataType == sl.MAT_TYPE.F32_C3: # float 3 channels
        imgMessage.encoding = ImageEncoding.TYPE_32FC3.value

    elif dataType == sl.MAT_TYPE.F32_C4: # float 4 channels
        imgMessage.encoding = ImageEncoding.TYPE_32FC4.value

    elif dataType == sl.MAT_TYPE.U8_C1: # unsigned char 1 channel
        imgMessage.encoding = ImageEncoding.MONO8.value

    elif dataType == sl.MAT_TYPE.U8_C2: # unsigned char 2 channels
        imgMessage.encoding = ImageEncoding.TYPE_8UC2.value

    elif dataType == sl.MAT_TYPE.U8_C3: # unsigned char 3 channels
        imgMessage.encoding = ImageEncoding.BGR8.value

    elif dataType == sl.MAT_TYPE.U8_C4: # unsigned char 4 channels
        imgMessage.encoding = ImageEncoding.BGRA8.value

    return imgMessage


def slTime2Ros(t: sl.Timestamp) -> rclpy.time.Time:
    """
    Convert a ZED timestamp to a ROS time.
    """
    return rclpy.time.Time(seconds=t.get_seconds(), nanoseconds=t.get_microseconds() * 1000)
