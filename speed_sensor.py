from machine import Pin
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

pin = Pin(5, Pin.IN, Pin.PULL_UP)
def callback(pin):
  global interrupt_flag
  interrupt_flag = 1
  
pin.irq(trigger = Pin.IRQ_FALLIING, handler = callback)
start_time = time.time()

while True:
  current_time = time.time()
  if interrupt_flag is 1:
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
    
    #print(a)
    # a = 0
    # start_time = time.time()


# previous_error = 0
# integral = 0
# i = 0
# I = 0
# setpoint = 20
# Kp = 1
# Ki = 1
# Kd = 1
# dt = 0.1

for (i < 10):
  error = (setpoint - previous_error)/ dt
  P = error
  I = I + (P * dt)
  D = (P - previous_error) / dt
  output = Kp * P + Ki * I + Kd * D
  previous_error = P
  time.sleep(dt)
