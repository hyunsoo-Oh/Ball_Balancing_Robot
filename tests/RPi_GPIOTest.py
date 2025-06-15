import RPi.GPIO as GPIO
import time

print(GPIO.VERSION)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

while True:
	GPIO.output(17, GPIO.HIGH)
	time.sleep(1)

	GPIO.output(17, GPIO.LOW)
	time.sleep(1)

GPIO.cleanup()