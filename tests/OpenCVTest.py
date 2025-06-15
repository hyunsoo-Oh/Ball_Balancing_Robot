# import cv2
# cap = cv2.VideoCapture(0)  # USB: 0, CSI는 보통 libcamera 지원 필요

# if cap.isOpened():
#     print("카메라 연결 성공")
# else:
#     print("카메라 연결 실패")

import numpy as np
import cv2
import io
import time
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

time.sleep(1)  # Warm-up

while True:
    frame = picam2.capture_array()
    cv2.imshow("Pi Camera Live", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()