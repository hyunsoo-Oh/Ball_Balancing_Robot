import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import cv2, numpy as np, time, threading, socket, random
import RPi.GPIO as GPIO
from utils.buffer import frame_queue
from app.camera import picam2_stream

# ──────────────── 서보모터 설정 ────────────────
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

SERVO_PINS = [17, 18, 27]
for pin in SERVO_PINS:
    GPIO.setup(pin, GPIO.OUT)

pwms = [GPIO.PWM(pin, 50) for pin in SERVO_PINS]
for pwm in pwms:
    pwm.start(0)

def set_servo_angle(servo_index, angle):
    if 0 <= servo_index < len(pwms):
        angle = max(0, min(180, angle))
        duty = 2 + (angle / 18)
        pwms[servo_index].ChangeDutyCycle(duty)
        time.sleep(0.05)
        pwms[servo_index].ChangeDutyCycle(0)

# ──────────────── UDP 설정 ────────────────
PC_IP = "10.10.10.95"
PC_PORT = 9090
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ──────────────── PID 평가 ────────────────
def evaluate_pid(Kp, Ki, Kd, duration=3.0):
    prev_error = 0
    integral = 0
    error_sum = 0
    count = 0
    start_time = time.time()

    try:
        while time.time() - start_time < duration:
            if frame_queue.empty():
                continue

            frame = frame_queue.get()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([10, 100, 100])
            upper = np.array([25, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
            blurred = cv2.GaussianBlur(mask, (9, 9), 2)

            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 100,
                                       param1=100, param2=30, minRadius=10, maxRadius=100)

            if circles is not None:
                x = int(circles[0][0][0])
                y = int(circles[0][0][1])
                error = 160 - x  # 중심 기준
                integral += error
                derivative = error - prev_error
                output = Kp * error + Ki * integral + Kd * derivative
                prev_error = error

                angle = 75 - output * 0.1
                angle = np.clip(angle, 0, 180)  # 안정화

                for i in range(3):
                    set_servo_angle(i, angle)

                # 제곱오차 + 변화량 패널티 방식
                error_sum += error**2 + 0.2 * abs(derivative)
                count += 1

                # 디버깅 출력
                print(f"[PID] err={error:>5.1f}, out={output:>6.1f}, angle={angle:>5.1f}")

                msg = f"{x},{y},{error:.2f},{Kp:.2f},{Ki:.2f},{Kd:.2f}"
                sock.sendto(msg.encode(), (PC_IP, PC_PORT))

                # 중심선 표시
                cv2.line(frame, (160, 0), (160, frame.shape[0]), (0, 255, 0), 1)
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            else:
                print("❌ 공 미검출")

            cv2.imshow("Camera", frame)
            cv2.waitKey(1)
            time.sleep(0.02)
    finally:
        return error_sum / count if count > 0 else float('inf')

# ──────────────── 메인 루프 ────────────────
if __name__ == "__main__":
    stream_thread = threading.Thread(target=picam2_stream, daemon=True)
    stream_thread.start()
    time.sleep(2)

    best_score = float('inf')
    best_params = (0, 0, 0)
    frequency = 50

    try:
        for i in range(frequency):
            Kp = random.uniform(0.1, 3.0)
            Ki = random.uniform(0.0, 0.1)
            Kd = random.uniform(0.05, 0.5)

            print(f"\n[{i+1}/{frequency}] Trying Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
            score = evaluate_pid(Kp, Ki, Kd, duration=10.0)
            print(f"→ 평균오차 Score = {score:.2f}")

            result_msg = f"RESULT,{Kp:.2f},{Ki:.2f},{Kd:.2f},{score:.2f}"
            sock.sendto(result_msg.encode(), (PC_IP, PC_PORT))

            if score < best_score - 0.01:
                best_score = score
                best_params = (Kp, Ki, Kd)
                print(f"✅ Best updated: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f} → Score={score:.2f}")
            if score < 10.0:
                print("🎯 임계값 도달 → 조기 종료!")
                break

    except KeyboardInterrupt:
        print("튜닝 중단됨")

    finally:
        for pwm in pwms:
            pwm.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print(f"\n🔚 최종 Best: Kp={best_params[0]:.2f}, Ki={best_params[1]:.2f}, Kd={best_params[2]:.2f}, Score={best_score:.2f}")
