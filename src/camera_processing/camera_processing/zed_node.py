#!/usr/bin/env python3

import cv2
import numpy as np
import pyzed.sl as sl

from threading import Lock, Thread
from time import sleep

import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer

from map_targets_between_cams import CameraUtil, Point, PitchYaw

lock = Lock()
run_signal = False
exit_signal = False
capture_thread = None
opt = None
viewer = None




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

def capture_ir_img_thread():
    global ir_image, run_ir_capture

    print("Initializing IR Cam Capture Thread...")

    try:
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, elp_width)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, elp_height)

        result, image = cam.read()
        print(f"IR Cam Dimensions: {image.shape}")

        while not exit_signal:
            try:
                result, image = cam.read()
                
                if (run_ir_capture):
                    lock.acquire()
                    ir_image = image
                    lock.release()
                    run_ir_capture = False
            except:
                lock.release()
            sleep(0.01)
    finally:
        cam.release()


def aruco_thread(is_zed: bool):
    global zed_left_image_net, ir_image, exit_signal, zed_detections, ir_cam_detections
    global run_zed_aruco, run_ir_aruco, run_ir_capture
    
    print(f"Initializing Aruco Thread. Zed: {is_zed}")

    if (is_zed):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    else:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

    aruco_params =  cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    zed_img_mapping = CameraUtil(2208, 1242, 110, 70)
    ir_img_mapping = CameraUtil(elp_width, elp_height, elp_hfov, elp_vfov)

    while not exit_signal:
        if (is_zed and run_zed_aruco) or (not is_zed and run_ir_aruco):
            lock.acquire()
            try:
                if (is_zed):
                    img = cv2.cvtColor(zed_left_image_net, cv2.COLOR_BGRA2RGB)
                else:
                    img = ir_image

                corners, ids, rejected = aruco_detector.detectMarkers(img)

                if (not is_zed):
                    cv2.aruco.drawDetectedMarkers(ir_image, corners, ids)

                detections = []

                if len(corners) == 0:
                    print("No aruco markers found")
                elif len(corners) != len(ids):
                    print("Mismatched id and corners arrays from the aruco detector")
                else:
                    # loop over the detected ArUCo corners
                    draw_corners = []
                    draw_markers = []
                    for i in range(0, len(corners)):
                        markerCorners = corners[i]
                        markerID = ids[i][0]

                        reshapedCorners = bounding_box(markerCorners.reshape((4, 2)))

                        if (not is_zed):
                            for points in reshapedCorners:
                                # print(f"    REMAPPING: {points}")
                                pitchYaw = ir_img_mapping.pitchYawFromXY(Point(points[0], points[1]))
                                xy = zed_img_mapping.xyFromPitchYaw(pitchYaw)
                                points[0] = max(0, xy.x)
                                points[1] = max(0, xy.y)
                                # print(f"           TO: {points}")
                                # print(f"     PitchYaw: {pitchYaw}")

                            draw_corners.append(reshapedCorners)
                            draw_markers.append(markerID)

                        # Creating ingestable objects for the ZED SDK
                        obj = sl.CustomBoxObjectData()
                        obj.unique_object_id = f"ArucoID{markerID}"
                        obj.bounding_box_2d = reshapedCorners # Converts to unsigned int so values MUST be positive
                        obj.label = int(markerID)
                        obj.probability = 0.99
                        obj.is_grounded = False
                        detections.append(obj)

                if (is_zed):
                    zed_detections = detections
                else:
                    ir_cam_detections = detections

                # if (not is_zed):
                #     try:
                #         img_zed = cv2.cvtColor(zed_left_image_net, cv2.COLOR_BGRA2RGB)
                        
                #         for corners_to_draw in draw_corners:
                #             (topLeft, topRight, bottomRight, bottomLeft) = corners_to_draw
                #             topRight = (int(topRight[0]), int(topRight[1]))
                #             bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                #             bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                #             topLeft = (int(topLeft[0]), int(topLeft[1]))

                #             # draw the bounding box of the ArUCo detection
                #             cv2.line(img_zed, topLeft, topRight, (0, 255, 0), 2)
                #             cv2.line(img_zed, topRight, bottomRight, (0, 255, 0), 2)
                #             cv2.line(img_zed, bottomRight, bottomLeft, (0, 255, 0), 2)
                #             cv2.line(img_zed, bottomLeft, topLeft, (0, 255, 0), 2)
                #             # compute and draw the center (x, y)-coordinates of the ArUco
                #             # marker
                #             cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                #             cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                #             cv2.circle(img_zed, (cX, cY), 4, (0, 0, 255), -1)
                #             # draw the ArUco marker ID on the image
                #             cv2.putText(img_zed, str(draw_markers[0]),
                #                 (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                #                 0.5, (0, 255, 0), 2)
                #             print(f"       CENTER:cX : {cX}, cY: {cY}")
                #     except:
                #         print("FAILED TO DRAW CORNERS")
                        
                #     cv2.imshow("zed", img_zed)
                #     key = cv2.waitKey(10)
                #     if key == 27:
                #         exit_signal = True

            finally:
                lock.release()

            if (is_zed):
                run_zed_aruco = False
            else:
                run_ir_aruco = False
        sleep(0.01)

def main():
    global zed_left_image_net, ir_image, exit_signal, zed_detections, ir_cam_detections
    global run_zed_aruco, run_ir_aruco, run_ir_capture

    run_zed_aruco = False
    run_ir_aruco = False
    run_ir_capture = False


    zed_aruco_thread = Thread(target=aruco_thread, kwargs={'is_zed': True})
    ir_cam_aruco_thread = Thread(target=aruco_thread, kwargs={'is_zed': False})
    ir_cam_capture_thread = Thread(target=capture_ir_img_thread)
    
    zed_aruco_thread.start()
    sleep(1)
    ir_cam_capture_thread.start()
    sleep(1)
    ir_cam_aruco_thread.start()
    sleep(3)

    print("Initializing Camera...")

    zed = sl.Camera()

    input_type = sl.InputType()
    if opt and opt.svo is not None:
        input_type.set_from_svo_file(opt.svo)

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
    init_params.camera_resolution = sl.RESOLUTION.HD2K   # HD1080   HD1200    HD2K
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = 20

    runtime_params = sl.RuntimeParameters()
    status = zed.open(init_params)

    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    image_left_tmp = sl.Mat()

    print("Initialized Camera")

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    zed.enable_object_detection(obj_param)

    objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

    # Display
    camera_infos = zed.get_camera_information()
    camera_res = camera_infos.camera_configuration.resolution
    
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    point_cloud_res = sl.Resolution(min(camera_res.width, 720), min(camera_res.height, 404))
    point_cloud_render = sl.Mat()
    viewer.init(camera_infos.camera_model, point_cloud_res, obj_param.enable_tracking)
    point_cloud = sl.Mat(point_cloud_res.width, point_cloud_res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)
    image_left = sl.Mat()
    
    # Utilities for 2D display
    display_resolution = sl.Resolution(min(camera_res.width, 1280), min(camera_res.height, 720))
    image_scale = [display_resolution.width / camera_res.width, display_resolution.height / camera_res.height]
    image_left_ocv = np.full((display_resolution.height, display_resolution.width, 4), [245, 239, 239, 255], np.uint8)

    # Utilities for tracks view
    camera_config = camera_infos.camera_configuration
    tracks_resolution = sl.Resolution(400, display_resolution.height)
    track_view_generator = cv_viewer.TrackingViewer(tracks_resolution, camera_config.fps, init_params.depth_maximum_distance)
    track_view_generator.set_camera_calibration(camera_config.calibration_parameters)
    image_track_ocv = np.zeros((tracks_resolution.height, tracks_resolution.width, 4), np.uint8)
    
    # Camera pose
    cam_w_pose = sl.Pose()

    while viewer.is_available() and not exit_signal:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # -- Get the image
            lock.acquire()
            zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
            zed_left_image_net = image_left_tmp.get_data()
            lock.release()

            # Run ZED aruco detections
            run_zed_aruco = True
            while run_zed_aruco:
                sleep(0.001)

            # Capture IR cam image
            run_ir_capture = True
            while run_ir_capture:
                sleep(0.001)

            # Run IR cam image
            run_ir_aruco = True
            while run_ir_aruco:
                sleep(0.001)


            # Wait for detections
            lock.acquire()
            # -- Ingest detections
            # zed.ingest_custom_box_objects(zed_detections)
            # zed.ingest_custom_box_objects(ir_cam_detections)
            zed.ingest_custom_box_objects(zed_detections + ir_cam_detections)
            lock.release()
            zed.retrieve_objects(objects, obj_runtime_param)

            # Print object detections
            # print(f"Objects: {objects.is_new} -> {repr(objects.object_list)}")
            for object in objects.object_list:
                print(f"Object ~ Id: {object.id}, position: {object.position}")

            # -- Display
            # Retrieve display data
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, point_cloud_res)
            point_cloud.copy_to(point_cloud_render)
            zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
            zed.get_position(cam_w_pose, sl.REFERENCE_FRAME.WORLD)

            # 3D rendering
            # viewer.updateData(point_cloud_render, objects)
            # 2D rendering
            np.copyto(image_left_ocv, image_left.get_data())
            cv_viewer.render_2D(image_left_ocv, image_scale, objects, obj_param.enable_tracking)
            global_image = cv2.hconcat([image_left_ocv, image_track_ocv])
            # Tracking view
            track_view_generator.generate_view(objects, cam_w_pose, image_track_ocv, objects.is_tracked)

            cv2.imshow("ZED | 2D View and Birds View", global_image)
            cv2.imshow("IR Cam Detections", ir_image)
            key = cv2.waitKey(5)
            if key == 27:
                exit_signal = True
        else:
            exit_signal = True

    viewer.exit()
    exit_signal = True
    zed.close()


if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--weights', type=str, default='yolov8m.pt', help='model.pt path(s)')
    # parser.add_argument('--svo', type=str, default=None, help='optional svo file')
    # parser.add_argument('--img_size', type=int, default=416, help='inference size (pixels)')
    # parser.add_argument('--conf_thres', type=float, default=0.4, help='object confidence threshold')
    # opt = parser.parse_args()

    try:
        main()
    except:
        cv2.destroyAllWindows()
        if viewer:
            viewer.close_func()