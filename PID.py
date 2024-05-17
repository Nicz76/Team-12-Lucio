import time
import numpy as np
import scipy.integrate import odeint

previous_error = 0
integral = 0
i = 0
I = 0
setpoint = 20
Kp = 1
Ki = 1
Kd = 1
dt = 0.1
for (i < 10):
  error = (setpoint - previous_error)/ dt
  P = error
  I = I + (P * dt)
  D = (P - previous_error) / dt
  output = Kp * P + Ki * I + Kd * D
  previous_error = P
  time.sleep(dt)
