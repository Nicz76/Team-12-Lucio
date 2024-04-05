import time
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

msg = ""
i = 0
alphabet = ["A","B","C","D","E","F"]
ser.write(alphabet[i].encode('utf-8'))
time.sleep(2)
while True:
    
    time.sleep(3)
    b=ser.readline()
    msg=b.decode('utf-8')
    if msg == alphabet[5]:
        print(msg)
        break
    if alphabet[i+1] == msg:
        print(msg)
        i+=2
        ser.write(alphabet[i].encode('utf-8'))
        time.sleep(2)
#     i+=1
#     print("Counter {} - Hello from Raspberry Pi".format(i))
#     ser.write('hello'.encode('utf-8'))
#     time.sleep(2)