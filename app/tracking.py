import cv2
import numpy as np
import time
from utils.buffer import frame_queue, tracking_data_queue

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
    prev_time = None
    prev_pos  = None
    prev_vel  = (0, 0)

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
