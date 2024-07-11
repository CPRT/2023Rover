import cv2
import os
from time import sleep

def main():
    i = 1
    camera = cv2.VideoCapture("/dev/video0")
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    try:
        while True:
            ret, img = camera.read()
            
            # cv2.imshow("Camera", img)
            cv2.imwrite(os.path.join('elp_ir_cam' , f'ELP_IR_CAM_{i}.jpg'), img)
            
            i+=1
            cv2.waitKey(0)

            print("Recording " + str(i))
            sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    main()