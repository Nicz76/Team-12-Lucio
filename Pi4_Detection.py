import RPi.GPIO as GPIO
import time
import numpy as np
import serial
import os
'''
ser = serial.Serial(
 port='/dev/ttyS0',
 baudrate = 9600,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=1
)
'''
'''
pwm1.duty_u16(9754)#1.5ms
pwm1.duty_u16(12354)#1.9ms
pwm1.duty_u16(7154)#1.1ms
ultrasonic_left = measure_distance(echo=17)
ultrasonic_front = measure_distance(echo=27)
ultrasonic_right = measure_distance(echo=22)

'''
# Define GPIO pinss
# The main trigger for all ultrasound sensor
TRIGGER_PIN = 4
# ECHO_PINS = [24, 25]   Add more GPIO pins if you have more sensors

def measure_distance(echo_pin):
    # Send a 10us pulse to trigger the measurement
    GPIO.output(TRIGGER_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIGGER_PIN, False)

    # Wait for the echo pin to go high and start the timer
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
    
    # Wait for the echo pin to go low and stop the timer
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()
    
    # Calculate the time difference and distance
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2  # Speed of sound is 34300 cm/s
    if (distance >= 100):
        return 'I' #Invaild, Distance is too long
    elif (distance <= 20):
        return 'D' #Danger, Object is nearby
    else:
        return 'S'#Safe, No object is nearby


#Creating an array to hold the values
send_arr = [0,0,0]

#Initialize values
while True:
    #Check left side
    send_arr[0] = measure_distance(17)
    #Check right side
    send_arr[2] = measure_distance(22)
    #Check front side
    send_arr[1] = measure_distance(27)
    #Convert it to bytes
    send_arr = send_arr.encode('utf-8')

    #Send the data
    #ser.write(send_arr)
    print(send_arr)
    time.sleep(0.5)
    time.sleep(dt)
