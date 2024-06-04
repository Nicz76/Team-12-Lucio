from gpiozero import DistanceSensor
#import RPi.GPIO as GPIO
#GPIO.setwarnings(False)
ultrasonic_left = DistanceSensor (echo=17, trigger=4)
ultrasonic_front = DistanceSensor (echo=27, trigger=6)
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
def PID_steer(distance_error):
    global I_steer
    global D_steer
    previous_error_steer = distance_error
    error_left = (setpoint - previous_error_steer)/ dt
    P = error_left
    I_steer = I_steer + (P * dt)
    D_steer = (P - previous_error_steer) / dt
    output = Kp * P + Ki * I_steer + Kd * D_steer
    output = -output
    #convert = (str(int(output))).encode('utf-8')
    send_arr[1] = int(output)
    return
def PID_straight():
    global previous_error_front
    global I_front
    global D_front
    previous_error_front = ultrasonic_front.distance
    error_front = (setpoint - previous_error_front)/ dt
    P = error_front
    I_front = I_front + (P * dt)
    D_front = (P - previous_error_front) / dt
    output = Kp * P + Ki * I_front + Kd * D_front
    #convert = (str(int(output))).encode('utf-8')
    send_arr[1] = int(output)

#Creating an array to hold the values
send_arr = [0,0]

#Initialize values
previous_error_front = 0
previous_error_steer = 0
I_steer = 0
I_striaght = 0
D_front = 0
D_steer = 0
setpoint = 0.30
Kp = 1
Ki = 1
Kd = 1
dt = 1
while True:
    front = ultrasonic_front.distance
    time.sleep(0.1)
    left = ultrasonic_left.distance
    time.sleep(0.1)
    right = ultrasonic_right.distance
    time.sleep(0.1) 
    if (front > setpoint):
        send_arr[0] = 1
    elif (send_arr[0] == 0):
        if ((left < setpoint) & (right < setpoint)):
            send_arr[0] = -1
            send_arr[1] = 0
        elif((left <= setpoint) & (right >= setpoint)):
            PID_steer(right)
        elif ((right <= setpoint) & (left >= setpoint)):
            left = -left
            PID_steer(left)
    elif(front <= setpoint):
        send_arr[0] = 0

    if((left < right) & (send_arr != -1)):
        PID_steer(right)
    elif ((right < left) & (send_arr != -1)):
        left = -left
        PID_steer(left)

    #Send the data
    print(send_arr)
    send_data = str(send_arr).encode('utf-8')
    #print(send_data)
    ser.write(send_data)
    #ser.write(send_arr[1])
    time.sleep(0.7)