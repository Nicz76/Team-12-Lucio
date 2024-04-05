# receiver.py / Tx/Rx => Tx/Rx
import os
import machine
from time import sleep
from machine import Pin,UART
import time

uart = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))
uart.init(bits=8, parity=None, stop=1)
print(uart)

b = None
i=0
msg = ""

while True:
    sleep(1)
    if uart.any():
        b = uart.readline()
        msg = b.decode('utf-8')
        print(msg)
        