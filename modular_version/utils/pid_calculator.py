from simple_pid import PID

pid = PID(0.5, 0.05, 0.01)
pid.output_limits = (0, 100)
pid.sample_time = 0.01
pid.auto_mode = True
pid.proportional_on_measurement = False
pid.differential_on_measurement = True

def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

def calculate_pid_output(kp, ki, kd, set_point, current_output, min_interval, max_interval, swap_intervals):
    pid.Kp = kp
    pid.Ki = ki
    pid.Kd = kd
    pid.setpoint = set_point

    output = pid(current_output)

    if swap_intervals:
        min_interval, max_interval = max_interval, min_interval

    high_time = map_range(output, 0, 100, min_interval, max_interval)
    low_time = map_range(output, 0, 100, max_interval, min_interval)

    PWM_enabled = high_time > 0

    return high_time, low_time, PWM_enabled, output
