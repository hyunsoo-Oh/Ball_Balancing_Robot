from picamera2 import Picamera2
import cv2
import time
from utils.buffer import frame_queue

# === Picamera2 초기화 ===
def picam2_init(width=640, height=480, fmt="RGB888"):
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (width, height)
    picam2.preview_configuration.main.format = fmt
    picam2.configure("preview")
    picam2.start()
    time.sleep(1)

    print("Picamera2 초기화 완료")
    return picam2

def picam2_stream():
    camera = picam2_init()

    while True:
        frame = camera.capture_array()
        if not frame_queue.full():
            frame_queue.put(frame)

def picam2_test():
    camera = picam2_init()
    while True:
        frame = camera.capture_array()
        cv2.imshow("Video", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    picam2_test()