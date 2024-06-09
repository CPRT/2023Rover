import cv2
import argparse
import numpy as np

aruco_dict = None
aruco_params = None
aruco_detector = None

def process(img, detector: cv2.aruco.ArucoDetector):

    res = detector.detectMarkers(img)
    print(repr(res))
    print()
    corners, ids, rejected = res

    print(f"ids: {repr(ids)}\ncorners: {repr(corners)}\nrejected: {repr(rejected)}")

    print()
    print(f"Desired Shape: {np.zeros((4, 2))}")
    print()

    for (markerCorners, markerId) in zip(corners, ids):
        print(f"Reshaped Corners: {repr(markerCorners.reshape((4, 2)))}\nId: {repr(markerId[0])}")

    print()
    print(f"Desired: {np.zeros((4, 2))}")

    print()
    print(f"Reshaped: {corners.reshape((4, 2))}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", type=str,
        default="",
        help="The image filename")
    args = vars(ap.parse_args())


    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params =  cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print(repr(aruco_detector))

    filename = args.get("image")
    print(f"Image: {repr(filename)}")
    img = cv2.imread(filename)
    process(img, aruco_detector)

    # try:
    #     img = cv2.imread()
    #     process(img)
            
    # except KeyboardInterrupt:
    #     print("Exiting now")


if __name__ == "__main__":
    try:
        main()
    finally:
        print("All done")




