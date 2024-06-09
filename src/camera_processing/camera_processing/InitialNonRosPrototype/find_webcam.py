import cv2

num = 0
# num = "/dev/media1"

print(f"Cam Num: {num}")

cam = cv2.VideoCapture(num)

if cam is None or not cam.isOpened():
    print(f"Warning: unable to open video source {num}")
    exit()


while True:
    result, image = cam.read()

    cv2.imshow(f"Cam Num: {num}", image)
    key = cv2.waitKey(10)
    if key == 27:
        break