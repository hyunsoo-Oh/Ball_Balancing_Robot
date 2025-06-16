import RPi.GPIO as GPIO
import time
import numpy as np
from utils.buffer import kinematics_data_queue

# === GPIO 핀 설정 (BCM 모드 기준) ===
SERVO_PINS = [17, 18, 27]  # 3개의 서보: 120도 간격

GPIO.setmode(GPIO.BCM)
for pin in SERVO_PINS:
    GPIO.setup(pin, GPIO.OUT)

# === 50Hz PWM 생성 (MG996R 기준) ===
pwms = [GPIO.PWM(pin, 50) for pin in SERVO_PINS]
for pwm in pwms:
    pwm.start(0) # 초기 Duty Cycle

# === 각도 → 듀티비 변환 함수 (MG996R 기준) ===
# 0도 → 2.5%, 180도 → 12.5%
def angle_to_duty(angle):
    return 2.5 + (angle / 180.0) * 10

# === 간단한 PID 클래스 ===
class PID:
    def __init__(self, kp, ki, kd, target=0.0):
        self.kp = kp    # 비례 게인
        self.ki = ki    # 적분 게인
        self.kd = kd    # 미분 게인
        self.target     = target  # 목표 위치
        self.integral   = 0       # 누적 오차 (I 항)
        self.prev_error = 0       # 이전 오차 (D 항 계산용)

    # 오차 계산 (현재 위치 - 목표 위치)
    def update(self, measurement):
        error = self.target - measurement
        self.integral += error
        derivative = error - self.prev_error

        # PID 출력 계산
        output = self.kp*error + self.ki*self.integral + self.kd*derivative

        # 상태 업데이트
        self.prev_error = error
        return output

# PID 게인 설정 (조정 필요)
pid_x = PID(kp=0.1, ki=0.0, kd=0.01, target=160)  # 예: 320x240 화면에서 중앙 160
pid_y = PID(kp=0.1, ki=0.0, kd=0.01, target=120)

# === 서보 방향 벡터 (120도 간격, 단위 벡터) ===
servo_dirs = [
    np.array([ 1.0,  0.0]),                   # 서보 1: 정면 (X축 방향)
    np.array([-0.5,  np.sqrt(3)/2]),          # 서보 2: 좌측 120°
    np.array([-0.5, -np.sqrt(3)/2])           # 서보 3: 우측 240°
]

# === 3서보 제어 함수 ===
def apply_control(ctrl_x, ctrl_y):
    roll  = ctrl_x   # X축 기준 제어량 → 좌우 기울기
    pitch = ctrl_y   # Y축 기준 제어량 → 앞뒤 기울기

    angles = []
    for i, vec in enumerate(servo_dirs):
        delta = vec[0] * roll + vec[1] * pitch
        angle = 90 + delta                    # 기준 각도 90°에서 ± 제어량
        angle = max(0, min(180, angle))       # 범위 제한
        angles.append(angle)
        pwms[i].ChangeDutyCycle(angle_to_duty(angle))

    print(f"🎯 서보 각도: {angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f}")

# === PID 제어 루프 ===
def pid_task():
    try:
        while True:
            data   = kinematics_data_queue.get()
            x, y   = data['x'], data['y']
            vx, vy = data['vx'], data['vy']
            ax, ay = data['ax'], data['ay']

            # PID 계산
            control_x = pid_x.update(x)
            control_y = pid_y.update(y)

            # 서보 제어
            apply_control(control_x, control_y)

    except KeyboardInterrupt:
        pass
    finally:
        for pwm in pwms:
            pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    pid_task()
