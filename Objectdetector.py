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
def PID_left():
    global previous_error_left
    global I_left
    global D_left
    previous_error_left = ultrasonic_left.distance
    error_left = (setpoint + previous_error_left)/ dt
    P = error_left
    I_left = I_left + (P * dt)
    D_left = (P - previous_error_left) / dt
    output = Kp * P + Ki * I_left + Kd * D_left
    #output = -output
    #convert = (str(int(output))).encode('utf-8')
    send_arr[1] = int(output)
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
Ki = 2
Kd = 1
dt = 0.5
while True:
    front = ultrasonic_front.distance
    time.sleep(0.1)
    left = ultrasonic_left.distance
    time.sleep(0.1)
    right = ultrasonic_right.distance
    time.sleep(0.1)
    '''
    if (front > setpoint):
        send_arr[0] = 1
    
    elif (send_arr[0] == 0):
        if ((left < setpoint) & (right < setpoint)):
            send_arr[0] = -1
            send_arr[1] = 0
        elif((left <= setpoint) & (right >= setpoint)):
            PID_right()
        elif ((right <= setpoint) & (left >= setpoint)):
            PID_left() 
    elif(front <= setpoint):
        send_arr[0] = 0

    if((left < right) & (send_arr != -1)):
        PID_right()
    elif ((right < left) & (send_arr != -1)):
        PID_left()
    '''
    if (left < setpoint) & (right < setpoint):
        send_arr[0] = -1
        send_arr[1] = 0
    elif left <= setpoint:
        PID_right()
        send_arr[0] = 1
    elif right <= setpoint:
        PID_left()
        send_arr[0] = 1
    elif front <= setpoint:
        send_arr[0] = 0
        ser.write(str(send_arr[0]).encode('utf-8'))
        time.sleep(1)
        if left < right:
            PID_right()
            send_arr[0] = 1
        else:
            PID_left()
            send_arr[0] = 1
    else:
        send_arr[0] = 1
    
    #Send the data
    print(send_arr)
    send_data = str(send_arr).encode('utf-8')
    #print(send_data)
    ser.write(send_data)
    #ser.write(send_arr[1])
    time.sleep(0.7)