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
alphabet = ["A","B","C","D","E","F"]

while True:
    sleep(1)
    if uart.any():
        b = uart.readline()
        msg = b.decode('utf-8')
        if alphabet[i] == msg:
            print(msg)
            uart.write(alphabet[i+1].encode('utf-8'))
            print("Message Send")
            i+=2
            time.sleep(2)
        
# #alphabet = ["A","B","C","D","E","F","G","H","I","K"]
# #i=0
# 
# #while i<10 :
#     
#    recieve = alphabet[i]
#    print(recieve)
#    i = i+1
#		print(type(msg))