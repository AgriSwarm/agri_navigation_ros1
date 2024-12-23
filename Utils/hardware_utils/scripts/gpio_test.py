# import RPi.GPIO as GPIO
import RPi.GPIO as GPIO
import time

output_pins = {
    'JETSON_XAVIER': 18,
    'JETSON_NANO': 33,
    'JETSON_NX': 33,
    'JETSON_ORIN_NX': 33,
}

print(GPIO.model)
output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
    raise Exception('PWM not supported on this board')

def main():
    print("PWM running. Press CTRL+C to exit.")
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    p = GPIO.PWM(output_pin, 490)
    p.start(15)

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            time.sleep(1)
            print("check")
    finally:
        p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
