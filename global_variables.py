import multiprocessing

manager = multiprocessing.Manager()

temp1 = manager.Value('d', 0.000)
temp2 = manager.Value('d', 0.000)
hum1 = manager.Value('d', 0.000)
hum2 = manager.Value('d', 0.000)

PID_kp = manager.Value('d', 12)
PID_ki = manager.Value('d', 0.01)
PID_kd = manager.Value('d', 0.2)
PID_set_point = manager.Value('d', 40)
PID_pv = manager.Value('d', 0.000)
PID_output = manager.Value('d', 0.000)

PWM_enabled = manager.Value('d', 1)
PWM_high_time = manager.Value('d', 0.000)
PWM_low_time = manager.Value('d', 0.000)

