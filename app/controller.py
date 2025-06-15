import RPi.GPIO as GPIO
import time

# GPIO 핀 설정 (BCM 모드 기준)
SERVO_PINS = [14]  # 사용할 핀 번호

GPIO.setmode(GPIO.BCM)
for pin in SERVO_PINS:
	GPIO.setup(pin, GPIO.OUT)

# 50Hz PWM 생성
pwms = [GPIO.PWM(pin, 50) for pin in SERVO_PINS]
for pwm in pwms:
	pwm.start(0) # 초기 Duty Cycle

# 각도 -> Duty Cycle 변환 함수
# 0도 → 2.5%, 180도 → 12.5%
def angle_to_duty(angle):
	return 2.5 + (angle / 180.0) * 10 

def servo_test():
	try:
		for angle in [0, 90, 180, 90]:
			print(f"각도 : {angle}°")
			for pwm in pwms:
				pwm.ChangeDutyCycle(angle_to_duty(angle))
			time.sleep(1)

	finally:
		for pwm in pwms:
			pwm.ChangeDutyCycle(0)
			pwm.stop()
		GPIO.cleanup()

if __name__ == "__main__":
	servo_test()