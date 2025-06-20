import threading
import time
from app.camera import picam2_stream
from app.tracking import tracking_task
from app.kinematics import kinematics_task
from app.controller import pid_task, initialize_servos
from app.random_search import random_pid_search

if __name__ == "__main__":
    initialize_servos()

    t1 = threading.Thread(target=picam2_stream, daemon=True)
    t2 = threading.Thread(target=kinematics_task, daemon=True)
    t3 = threading.Thread(target=pid_task, daemon=True)

    t1.start()
    t2.start()
    t3.start()

    try:
        while True:
            tracking_task()
            # tracking_task_yolo()

    except KeyboardInterrupt:
        print("프로그램 종료 요청")
