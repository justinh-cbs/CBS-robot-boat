import Jetson.GPIO as GPIO
import time

# Test just a single pin
TEST_PIN = 32  # Change this to whichever pin you want to test

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TEST_PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(TEST_PIN, GPIO.HIGH)
        print(f"Pin {TEST_PIN} HIGH")
        time.sleep(3)  # Long delay to make measurement easier
        GPIO.output(TEST_PIN, GPIO.LOW)
        print(f"Pin {TEST_PIN} LOW")
        time.sleep(3)
except KeyboardInterrupt:
    GPIO.cleanup()
    print("\nExiting cleanly")
