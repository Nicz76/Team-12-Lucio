from machine import Pin, Timer
import time
a = 0
interrupt_flag = 0

previous_error = 0
integral = 0
i = 0
I = 0
setpoint = 20
Kp = 1
Ki = 1
Kd = 1
dt = 0.1
interrupt_count = 0
lastTime = time.time_ns()
print(lastTime)
pin = Pin(9, Pin.IN, Pin.PULL_UP)
sensor = machine.Pin(9, machine.Pin.IN)


def handle_interrupt(pin):
  global interrupt_count
  global lastTime
  interval = time.time_ns() - lastTime
  if (interval > 1000000):
    lastTime = time.time_ns()
    if(sensor.value()==1):
      interrupt_count = interrupt_count + 1
sensor.irq(trigger = Pin.IRQ_RISING, handler = handle_interrupt)

def callback(pin):
  global interrupt_count
  interrupt_flag = interrupt_flag + 1
  
def periodicFunction(pin):
    global interrupt_count
    print(f"Interrupt count:" + str(interrupt_count))
    interrupt_count = 0
    

Timer(mode=Timer.PERIODIC, freq=1, callback=periodicFunction)

start_time = time.time()
    
'''
while True:
  current_time = time.time()
  if interrupt_flag == 1:
    a = a + 1
    interrupt_flag = 0.
  if current_time - start_time >= dt:
    error = setpoint - previous_error
    P = error
    integral += P*dt
    D = (P - previous_error) / dt
    output = Kp * P +Ki * integral + Kd * D
    print(f"PID Output: {output}")
    previous_error = error
    if current_time - start_time >= 1:
      print(f"Interrupt count: {a}")
      a = 0
      start_time = time.time()
    start_time = current_time

time.sleep(0.01)
'''
#for (i < 10):
 # error = (setpoint - previous_error)/ dt
  #P = error
  #I = I + (P * dt)
  #D = (P - previous_error) / dt
  #output = Kp * P + Ki * I + Kd * D
  #previous_error = P
  #time.sleep(dt)
