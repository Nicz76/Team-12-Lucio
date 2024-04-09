import RPi.GPIO as GPIO
import time

# Set GPIO mode (BOARD or BCM)
GPIO.setmode(GPIO.BOARD)

# Define GPIO pins for Trigger and Echo
TRIG = 7
ECHO = 11

# Set up GPIO pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def distance():
    # Ensure trigger is off
    GPIO.output(TRIG, False)
    time.sleep(0.5)

    # Send 10us pulse to trigger
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Measure time of pulse
    pulse_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 17150
    distance_in = distance_cm / 2.54

    return distance_cm, distance_in

try:
    while True:
        dist_cm, dist_in = distance()
        print("Distance: {:.2f} cm, {:.2f} inches".format(dist_cm, dist_in))
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()