import sys
import os
import threading
import time
import cv2
import tensorflow as tf

# 프로젝트 루트 디렉토리(BBROBOT)를 Python 경로에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

MODEL_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "yolo_model", "yolov5n_flex.tflite"))
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"[ERROR] 모델 파일을 찾을 수 없습니다: {MODEL_PATH}")

interpreter = tf.lite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

from queue import Queue
from app.camera import picam2_stream
from utils.buffer import frame_queue
from app.tracking import tracking_task_yolo  # ← 위에서 제공한 코드가 tracking_yolo.py에 저장되었다고 가정

if __name__ == "__main__":
    # 프레임 수신 스레드 시작
    camera_thread = threading.Thread(target=picam2_stream, daemon=True)
    camera_thread.start()

    # 카메라 안정화 대기
    print("[INFO] 카메라 초기화 중...")
    time.sleep(2)

    # YOLO 기반 추적 테스트 시작
    print("[INFO] YOLO 추적 시작")
    tracking_task_yolo()
