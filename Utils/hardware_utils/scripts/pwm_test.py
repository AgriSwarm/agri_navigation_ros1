import Jetson.GPIO as GPIO
import time
import threading

output_pin = 32
input_pin = 31

def pwm_output():
    p = GPIO.PWM(output_pin, 50)
    p.start(80)
    try:
        while True:
            time.sleep(1)
    finally:
        p.stop()

def pwm_input():
    while True:
        high_time = 0
        low_time = 0
        
        # Measure high time
        while GPIO.input(input_pin) == GPIO.LOW:
            pass
        start_time = time.time()
        while GPIO.input(input_pin) == GPIO.HIGH:
            high_time = time.time() - start_time
        
        # Measure low time
        while GPIO.input(input_pin) == GPIO.HIGH:
            pass
        start_time = time.time()
        while GPIO.input(input_pin) == GPIO.LOW:
            low_time = time.time() - start_time
        
        total_time = high_time + low_time
        frequency = 1 / total_time
        duty_cycle = (high_time / total_time) * 100
        
        print("Frequency: {:.2f} Hz".format(frequency))
        print("Duty Cycle: {:.2f}%".format(duty_cycle))
        time.sleep(1)

def digital_output():
    while True:
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(output_pin, GPIO.LOW)
        time.sleep(1)

def digital_input():
    while True:
        state = GPIO.input(input_pin)
        if state == GPIO.HIGH:
            print("Input is HIGH")
        else:
            print("Input is LOW")
        time.sleep(0.5)

def main():
    print("Digital I/O test running. Press CTRL+C to exit.")
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(input_pin, GPIO.IN)

    output_thread = threading.Thread(target=pwm_output)
    input_thread = threading.Thread(target=pwm_input)

    output_thread.daemon = True
    input_thread.daemon = True

    output_thread.start()
    input_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting program")
    finally:
        GPIO.cleanup()
        print("\nCleanup complete.")

if __name__ == '__main__':
    main()
