#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time
import sys

def control_motor(duration, pin=32):
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(pin, GPIO.OUT)
        
        pwm = GPIO.PWM(pin, 50)
        pwm.start(30)
        
        time.sleep(float(duration))
        
        pwm.stop()
        GPIO.cleanup()
        return True
    except Exception as e:
        print(f"Error: {str(e)}")
        GPIO.cleanup()
        return False

if __name__ == '__main__':
    if len(sys.argv) > 1:
        control_motor(float(sys.argv[1]))
