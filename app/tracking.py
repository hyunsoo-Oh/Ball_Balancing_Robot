import cv2
import numpy as np
import time
from utils.buffer import frame_queue, tracking_data_queue

# # === YOLOv5n TFLite 모델 로드 ===
# interpreter = tf.lite.Interpreter(model_path="yolo_model/yolov5n_flex.tflite")
# interpreter.allocate_tensors()
# input_details = interpreter.get_input_details()
# output_details = interpreter.get_output_details()

# # 모델 입력 크기
# input_shape = input_details[0]['shape'][1:3]  # [height, width]

# # 클래스 이름 (YOLO 학습 시 사용한 class 순서)
# CLASSES = ["yellow_ball"]  # Roboflow에서 지정한 클래스 이름

# # === YOLOv5 TFLite 공 검출 함수 ===
# def detect_ball_yolo(frame):
#     img = cv2.resize(frame, tuple(input_shape))
#     img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     input_data = np.expand_dims(img_rgb, axis=0).astype(np.float32) / 255.0

#     interpreter.set_tensor(input_details[0]['index'], input_data)
#     interpreter.invoke()
#     output_data = interpreter.get_tensor(output_details[0]['index'])[0]

#     h, w, _ = frame.shape
#     boxes = []

#     for det in output_data:
#         x_center, y_center, width, height, conf, cls = det
#         if conf < 0.4:
#             continue
#         class_id = int(cls)
#         if CLASSES[class_id] != "yellow_ball":
#             continue

#         # 좌표 원본 이미지 크기에 맞게 변환
#         x = int((x_center - width / 2) * w)
#         y = int((y_center - height / 2) * h)
#         x2 = int((x_center + width / 2) * w)
#         y2 = int((y_center + height / 2) * h)
#         boxes.append((x, y, x2, y2, conf))

#     return boxes

# # === 추적 메인 함수 ===
# def tracking_task_yolo():
#     while True:
#         frame = frame_queue.get()
#         detections = detect_ball_yolo(frame)
#         curr_time = time.time()

#         if detections:
#             # 가장 큰 객체 기준
#             largest = max(detections, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
#             x1, y1, x2, y2, conf = largest
#             x, y = (x1 + x2) // 2, (y1 + y2) // 2

#             data = {
#                 'x': x,
#                 'y': y,
#                 'timestamp': curr_time
#             }
#             if not tracking_data_queue.full():
#                 tracking_data_queue.put(data)

#             # 시각화
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#             cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
#             cv2.putText(frame, f"({x}, {y})", (x + 10, y - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

#         cv2.imshow("YOLOv5n Ball Tracking", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cv2.destroyAllWindows()

# === 공 검출 함수 ===
def detect_ball(frame):
    # BGR → HSV 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 공 색 범위 설정 (주황색)
    lower_orange = np.array([10, 100, 100]) # 범위 조정
    upper_orange = np.array([25, 255, 255]) # 범위 조정
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Mask 정제
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # 윤곽선 검출 Edge Detect
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # 가장 큰 윤곽선 선택
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)

        if radius > 10: # 작은 노이즈 제거
            center = (int(x), int(y))
            cv2.circle(frame, center, int(radius), (0, 255, 0), 2) # 공의 외곽
            cv2.circle(frame, center, 5, (0, 0, 255), -1)            # 중심점
            return int(x), int(y), mask

    # 공이 감지되지 않으면 None 반환
    return None, None, mask

# === Frame에서 공을 추적하고 좌표 출력 ===
def tracking_task():
    while True:
        frame = frame_queue.get()
        result_x, result_y, mask = detect_ball(frame)
        curr_time = time.time()
        # 좌표 출력
        if result_x is not None:
            x, y = result_x, result_y
            data = {
                'x': x,
                'y': y,
                'timestamp': curr_time
            }
            if not tracking_data_queue.full():
                tracking_data_queue.put(data)

            # 좌표 텍스트 영상 위에 표시
            text = f"({x}, {y})"
            cv2.putText(frame, text, (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # cv2.imshow("HSV Mask", mask)                # 마스크 디버깅용
        cv2.imshow("Ball Tracking", frame)  # 추적 결과
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 종료 처리
    cv2.destroyAllWindows() # OpenCV로 띄운 영상 창을 모두 닫음

if __name__ == "__main__":
    tracking_task()
