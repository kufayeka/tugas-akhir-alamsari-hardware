import multiprocessing

manager = multiprocessing.Manager()

temp1 = manager.Value('d', 0.000)
temp2 = manager.Value('d', 0.000)
hum1 = manager.Value('d', 0.000)
hum2 = manager.Value('d', 0.000)

PID_kp = manager.Value('d', 8.1) # 30 * 0.6 = 18
PID_ki = manager.Value('d', 2.57) # 30 * 1.2 / 420 = 0,086
PID_kd = manager.Value('d', 1.13) # 30 * 420 * 0.075 = 945
PID_set_point = manager.Value('d', 33)
PID_pv = manager.Value('d', 0.000)
PID_output = manager.Value('d', 0.000)

PWM_enabled = manager.Value('d', 1)
PWM_max_interval = manager.Value('d', 5)
PWM_min_interval = manager.Value('d', 0)
PWM_high_time = manager.Value('d', 0.000)
PWM_low_time = manager.Value('d', 0.000)

MQTT_is_connected = manager.Value('d', 0)

