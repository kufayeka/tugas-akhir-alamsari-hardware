from simple_pid import PID

pid = PID(10, 0.01, 0.1)
pid.output_limits = (0, 100)
pid.sample_time = 0.01
pid.auto_mode = True
pid.proportional_on_measurement = False
pid.differential_on_measurement = True

def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

def calculate_pid_output(kp, ki, kd, set_point, current_output, interval_time_from, interval_time_to, swap_intervals):
    pid.Kp = kp
    pid.Ki = ki
    pid.Kd = kd
    pid.setpoint = set_point

    output = pid(current_output)

    if swap_intervals:
        interval_time_from, interval_time_to = interval_time_to, interval_time_from

    high_time = map_range(output, 0, 100, interval_time_from, interval_time_to)
    low_time = map_range(output, 0, 100, interval_time_to, interval_time_from)

    if high_time > 0:
        PWM_enabled = True
    else: 
        PWM_enabled = False

    return high_time, low_time, PWM_enabled, output
