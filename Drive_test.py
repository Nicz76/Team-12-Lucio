import machine
import time;
from machine import Pin, PWM;
from machine import UART;
from time import sleep;

uart = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))
uart.init(bits=8, parity=None, stop=1)
print(uart)

#Steering and acclerate respectively
#duty cycle u16 = time duty cycle * 65535
def forward():
    ina.value(1)
    inb.value(0)
def backward():
    ina.value(0)
    inb.value(1)
def brake():
    ina.value(0)
    inb.value(0)
def left():
    pwm1.duty_u16(7154)#1.1ms
def right():
    pwm1.duty_u16(12354)#1.9ms
def straight():
    pwm1.duty_u16(9754)#1.5ms

#Ultrasound Sensor
def measure_distance():
    # Send a short pulse to trigger pin
    TRIG_PIN.on()
    time.sleep_us(10)
    TRIG_PIN.off()
    # Wait for the echo pin to go high
    while not ECHO_PIN.value():
        pass
    pulse_start = time.ticks_us()
    # Wait for the echo pin to go low
    while ECHO_PIN.value():
        pass
    pulse_end = time.ticks_us()
    # Calculate duration of pulse
    pulse_duration = time.ticks_diff(pulse_end, pulse_start)
    # Speed of sound in cm/us (approx. 0.0343 cm/us at room temperature)
    speed_of_sound = 0.0343
    # Calculate distance (distance = speed * time / 2)
    distance = (pulse_duration * speed_of_sound) / 2
    return distance

def dodge():
    brake()
    time.sleep(1)
    pwm3.duty_u16(7154)#cam_angle = -45
    dist_left = measure_distance()
    time.sleep(1)
    pwm3.duty_u16(9754)#cam_angle = 0
    dist_straight = measure_distance()
    time.sleep(1)
    pwm3.duty_u16(12354)#cam_angle = 45
    dist_right = measure_distance()
    time.sleep(1)
    #Perform a U-turn
    if (dist_straight <= 50 & dist_right <= 50 & dist_left <= 50):
        backward()
        time.sleep(1)
        left()
        forward()
        time.sleep(3)
        straight()
    elif (dist_left < dist_right): #check if there is something on the right
        dodge_right()
    elif (dist_right < dist_left): #check if there is something on the left
        dodge_left()

#If the object is on the right side
def dodge_left():
    brake()
    time.sleep(1)
    left()
    time.sleep(1)
    forward()
    time.sleep(1)
    straight()
    time.sleep(1)
    right()
    time.sleep(3)
    straight()
    time.sleep(1)
    left()
    time.sleep(1)
    straight()
    brake()

#If the object is on the left side
def dodge_right():
    brake()
    time.sleep(1)
    right()
    time.sleep(1)
    forward()
    time.sleep(1)
    straight()
    time.sleep(1)
    left()
    time.sleep(3)
    straight()
    time.sleep(1)
    right()
    time.sleep(1)
    straight()
    brake()

'''
def measure_space_left #135 degrees

def measure_space_right #45 degrees

def dodge_obstacle(): 
    dist = measure_distance()
    if dist <= 50:
        space_left = measure_space_left()
        space_right = measure_space_right()
        
        if space_left > space_right:
            dodge_left()
        else:
            dodge_right()
'''

# Define GPIO pins
TRIG_PIN = Pin(14, machine.Pin.OUT)
ECHO_PIN = Pin(15, machine.Pin.IN)
pwm1 = PWM(Pin(0)) #Servo
pwm2 = PWM(Pin(1)) #Speed
pwm3 = PWM(Pin(15)) #Camera angle
pwm2.duty_u16(8000) #Acceleration
ina = Pin(2, Pin.OUT)
inb = Pin(3, Pin.OUT)
pwm1.freq(100)
dist_right = 200
dist_left = 200
dist_straight = 200
#Reading the data from Pi 4 using UART
b = None
msg = ""
cam_angle = 45
while True:
    sleep(1)
    if uart.any():
        #Need to find the optimal read
        b = uart.readline()
        msg = b.decode('utf-8')
        print(msg)
        #uart.write(alphabet[i+1].encode('utf-8'))
        print("Message Recieved")
        time.sleep(2)
        if msg == "w":
            forward()
        elif msg == "s":
            backward()
        elif msg == "a":
            left()
        elif msg == "d":
            right()
        elif msg == "x":
            brake()
        #If the car stalls
        elif msg == "p":
            brake()
            time.sleep(10)
#    dist = measure_distance()
#   print("Distance: {:.2f} cm".format(dist))
    time.sleep(1)
    pwm3.duty_u16(7154)#cam_angle = -45
    dist_left = measure_distance()
    time.sleep(1)
    pwm3.duty_u16(9754)#cam_angle = 0
    dist_straight = measure_distance()
    time.sleep(1)
    pwm3.duty_u16(12354)#cam_angle = 45
    dist_right = measure_distance()
    time.sleep(1)
    if (dist_right <= 50): #check if there is something on right
        dodge_left()
    if (dist_left <= 50): #check if there is something on left
        dodge_right() 
    if (dist_straight <=50):
        dodge()
#End of reading  
