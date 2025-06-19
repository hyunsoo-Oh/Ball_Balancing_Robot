from picamera2 import Picamera2
import cv2
import time
from utils.buffer import frame_queue

# === Picamera2 초기화 ===
def picam2_init(width=2400, height=2400, fmt="RGB888"):
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (width, height)
    picam2.preview_configuration.main.format = fmt
    picam2.configure("preview")
    picam2.start()
    time.sleep(1)

    print("Picamera2 초기화 완료")
    return picam2

def picam2_full_fov():
    picam2 = Picamera2()

    # Mode 1 (1640x1232) 사용 - 전체 센서 범위에서 다운샘플링
    config = picam2.create_preview_configuration(
        main={"size": (1640, 1232), "format": "RGB888"},
        controls={"FrameDurationLimits": (8333, 8333)}
    )
    picam2.configure(config)

    # 전체 센서 크롭 설정
    picam2.set_controls({
        "ScalerCrop": (0, 0, 3280, 2464)  # 센서 해상도와 동일
    })

    picam2.start()
    time.sleep(2)

    return picam2

def picam2_stream():
    # camera = picam2_init()
    camera = picam2_full_fov()

    target_width = 320  # 원하는 폭으로 변경 가능

    while True:
        frame = camera.capture_array()

        # 비율 유지하면서 리사이징
        scale = target_width / frame.shape[1]
        target_height = int(frame.shape[0] * scale)
        frame_resized = cv2.resize(frame, (target_width, target_height))

        if not frame_queue.full():
            frame_queue.put(frame_resized)

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
