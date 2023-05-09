import multiprocessing

manager = multiprocessing.Manager()

temp1 = manager.Value('d', 0.000)
temp2 = manager.Value('d', 0.000)
hum1 = manager.Value('d', 0.000)
hum2 = manager.Value('d', 0.000)

PID_kp = manager.Value('d', 0.000)
PID_ki = manager.Value('d', 0.000)
PID_kd = manager.Value('d', 0.000)

kp = 4.83
ki = 0.0201225
kd = 0.0805
sp = 61

temp1 = manager.Value('d', 0.000)
temp2 = manager.Value('d', 0.000)
hum1 = manager.Value('d', 0.000)
hum2 = manager.Value('d', 0.000)

pid_output = manager.dict({
    'pwm_enabled': True,
    'high_time': 0,
    'low_time': 0,
})

sensors_climate_readings = manager.dict({
    'temp1': 0,
    'temp2': 0,
    'hum1': 0,
    'hum2': 0,
})


pid_output = manager.dict({
    'pwm_enabled': True,
    'high_time': 0,
    'low_time': 0,
    'output': 0,
})

pid_parameters = manager.dict({
    'kp': kp,
    'ki': ki,
    'kd': kd,
    'pv': 0,
    'set_point': sp,
})