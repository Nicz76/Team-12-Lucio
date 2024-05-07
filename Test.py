import time;
from machine import Pin, PWM;
from machine import UART;
from time import sleep;

uart = UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

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

pwm1 = PWM(Pin(0)) #Servo
pwm2 = PWM(Pin(1)) #Speed
pwm2.duty_u16(4120)
ina = Pin(2, Pin.OUT)
inb = Pin(3, Pin.OUT)
pwm1.freq(100)
command ='w'
while True:
        
    pwm1.duty_u16(9754)#1.5ms
    #pwm2.duty_u16(9754)#1.5ms
    ina.value(1)
    inb.value(0)
    time.sleep(2)
    pwm1.duty_u16(7154)#1.1ms
    #pwm2.duty_u16(7154)#1.1ms
    ina.value(0)
    inb.value(0)
    time.sleep(2)
    pwm1.duty_u16(9754)#1.5ms
    ina.value(0)
    inb.value(1)
    time.sleep(2)
    pwm1.duty_u16(12354)#1.9ms
    ina.value(0)
    inb.value(0)
    time.sleep(2)
    pwm1.duty_u16(9754)#1.5ms
    time.sleep(5)
    pwm1.duty_u16(9754)#1.5ms
    time.sleep(5)
    pwm1.duty_u16(9754)#1.5ms
    time.sleep(5)
    pwm1.duty_u16(9754)#1.5ms
    time.sleep(5)
    pwm1.duty_u16(9754)#1.5ms
    time.sleep(5)