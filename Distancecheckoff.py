from gpiozero import DistanceSensor
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
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
pwm1.duty_u16(12354)#1.9ms
pwm1.duty_u16(7154)#1.1ms
'''
def PID_left():
    global previous_error_left
    global I_left
    global D_left
    previous_error_left = ultrasonic_left.distance
    error_left = (setpoint - previous_error_left)/ dt
    P = error_left
    I_left = I_left + (P * dt)
    D_left = (P - previous_error_left) / dt
    output = Kp * P + Ki * I_left + Kd * D_left
    previous_error_left = P
    convert = (str(int(output))).encode('utf-8')
    send_arr[1] = convert
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
    previous_error_right = P
    convert = (str(int(output))).encode('utf-8')
    send_arr[1] = convert
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
    previous_error_front = P
    convert = (str(int(output))).encode('utf-8')
    send_arr[1] = convert

#Creating an array to hold the values
send_arr = [0,0]

#Initialize values
previous_error_front = 0
previous_error_left = 0
previous_error_right = 0
I_left = 0
I_right = 0
I_striaght = 0
D_front = 0
D_left = 0
D_right = 0
setpoint = 0.30
Kp = 1
Ki = 1
Kd = 1
dt = 0.1
while True:
    if(ultrasonic_front <= (setpoint)):
        brake = 1
        if (ultrasonic_left < setpoint & ultrasonic_right < setpoint):
            U_turn = 1
        elif(ultrasonic_left > ultrasonic_right):
            PID_left()
        else:
            PID_right()
    elif(ultrasonic_left <= setpoint):
        PID_right()
    elif (ultrasonic_right <= setpoint):
        PID_left()

    #Send the data
    ser.write(send_arr)
    print(send_arr)
    time.sleep(1)
    time.sleep(dt)