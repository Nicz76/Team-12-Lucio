import time

previous_error = 0
integral = 0
i = 0
setpoint = 20
dt = 0.1
for (i < 10):
  error = (setpoint - previous error)/ dt
  P = error
  I = I + (P * dt)
  D = (P-pervious_error) / dt
  output = Kp * P + Ki * I + Kd * D
  previous_error = P
  time.sleep(dt)