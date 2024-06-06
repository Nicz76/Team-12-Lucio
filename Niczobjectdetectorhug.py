from gpiozero import DistanceSensor
#import RPi.GPIO as GPIO
#GPIO.setwarnings(False)
ultrasonic_left = DistanceSensor (echo=17, trigger=4)
ultrasonic_right = DistanceSensor (echo=22, trigger=5)
import time
import numpy as np
import serial
import os

ser = serial.Serial(
 port='/dev/ttyS0',
 baudrate = 9600,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=1
)

'''
pwm1.duty_u16(9754)#1.5ms
pwm1.duty_u16(12354)#1.9ms (Right)
pwm1.duty_u16(7154)#1.1ms (Left)
'''
def PID_left():
    global previous_error_left
    global I_left
    global D_left
    previous_error_left = ultrasonic_left.distance
    error_left = (setpoint - previous_error_left)/ dt
    P = P - error_left
    I_left = I_left + (P * dt)
    D_left = (P - previous_error_left) / dt
    output = Kp * P + Ki * I_left + Kd * D_left
    #output = -output
    #convert = (str(int(output))).encode('utf-8')
    send_arr = int(output)
    return
def PID_right():
    global previous_error_right
    global I_right
    global D_right
    previous_error_right = ultrasonic_right.distance
    error_right = (setpoint - previous_error_right)/ dt
    P = error_right
    I_right = I_right + (P * dt)
    D_right = (P - previous_error_right) / dt
    output = Kp * P + Ki * I_right + Kd * D_left
    #convert = (str(int(output))).encode('utf-8')
    send_arr = int(output)
    return
#Create a variable to hold the value
send_arr = 0

#Initialize values
previous_error_left = 0
previous_error_right = 0
I_left = 0
I_right = 0
D_front = 0
D_left = 0
D_right = 0
setpoint = 0.50
Kp = 1
Ki = 2
Kd = 1
dt = 0.5
count = 0
while True:
    left = ultrasonic_left.distance
    time.sleep(0.1)
    right = ultrasonic_right.distance
    time.sleep(0.1)
    if (right <= 1):
        
    if(right < setpoint):
        PID_left()
    else:
        PID_right()
    if (count <= 1):
        count = 0
    elif ((left < setpoint) & (right < setpoint) & (count == 0)):
        send_arr = 0
        count += 1
        
    #Send the data
    print(send_arr)
    send_data = str(send_arr).encode('utf-8')
    #print(send_data)
    ser.write(send_data)
    #ser.write(send_arr[1])
    time.sleep(0.7)