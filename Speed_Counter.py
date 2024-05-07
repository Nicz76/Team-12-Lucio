import time
from machine import Pin

a=0
interrupt_flag=0
#create an input pin on pin #0
pin = Pin(5, Pin.IN, Pin.PULL_UP)
def callback(pin):
    global interrupt_flag
    interrupt_flag=1

#time.init(period=1000, callback=mycallback)

pin.irq(trigger=Pin.IRQ_FALLING, handler=callback)
start_time = time.time()
while True:
    current_time = time.time()
    if interrupt_flag is 1:
        a = a + 1
        interrupt_flag = 0
    if current_time - start_time >= 1:
        print(a)
        a = 0
        start_time = time.time()
    time.sleep(1)
#Initialize variable
#tim.init(mode=Timer.PERIODIC, period=1000, callback=mycallback)
#num_count = 0
#Counting the number of falling edges interrupts in a time interval of 1 sec
    
#Pringt the number of counts per second
#print ("Car's Speed is ", num_count)
#num_count = 0