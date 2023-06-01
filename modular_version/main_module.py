import datetime
import time
from utils import climate_sensors
from utils import pid_calculator

sensor1_humidity = 0
PID_kp, PID_ki, PID_kd = 0.5, 0.05, 0.01
PID_set_point = 90
PID_output = 0
PWM_enabled = False
PWM_high_time = 1   # Dynamic interval for relay_on()
PWM_low_time = 2    # Dynamic interval for relay_off()
on = 10
off = 5
min_interval, max_interval, swap_intervals = 0, 5, False

def read_sensor(ts):
    global sensor1_humidity

    sensors_readings = climate_sensors.read_climate_sensors()
    sensor1_temperature = sensors_readings['sensor1']['temperature']
    sensor1_humidity = sensors_readings['sensor1']['humidity']
    sensor2_temperature = sensors_readings['sensor2']['temperature']
    sensor2_humidity = sensors_readings['sensor2']['humidity']

    print("____________________________________")
    print(f"Current Sensor Readings: ({ts})")
    print(f"\tTemp 1:{sensor1_temperature:.3f} | Hum 1:{sensor1_humidity:.3f}")
    print(f"\tTemp 2:{sensor2_temperature:.3f} | Hum 2:{sensor2_humidity:.3f}")
    print()
    print(f"PID Parameters: kp:{PID_kp} | ki:{PID_ki} | kd:{PID_kd}")
    print("PID PV:", sensor1_humidity)
    print("PID Set Point:", PID_set_point)
    print(f"PID Output: {PID_output}")
    print()
    print("PWM Enabled:", PWM_enabled)
    print("PWM Timing / DutyCycle (secs): ")
    print(f"\tHigh Time:{PWM_high_time:.3f} | Low Time:{PWM_low_time:.3f}")
    print()

def record_data(ts):
    print("record sensor data...", ts, sensor1_humidity)

def relay_on(ts):
    print("RELAY ON", ts)

def relay_off(ts):
    print("RELAY OFF", ts)

if __name__ == '__main__':
    sensor_interval = 1     # 1 second interval for read_sensor()
    data_interval = 10      # 10 second interval for record_data()

    sensor_last_time = time.time()
    data_last_time = time.time()

    try:
        while True:
            timestamp = datetime.datetime.now().strftime('%H:%M:%S')
            # Read the sensor data every 1 second
            if time.time() - sensor_last_time >= sensor_interval:
                read_sensor(timestamp)
                sensor_last_time = time.time()

            # Record the data every 10 seconds
            if time.time() - data_last_time >= data_interval:
                record_data(timestamp)
                data_last_time = time.time()
                
            # Continuously calculate PID
            PWM_high_time, PWM_low_time, PWM_enabled, PID_output = pid_calculator.calculate_pid_output(PID_kp, PID_ki, PID_kd, PID_set_point, sensor1_humidity, min_interval, max_interval, swap_intervals)

    except KeyboardInterrupt:
        print("\n\nKeyboardInterrupt detected. Exiting...")

    finally:
        # Cleanup code, if any
        pass
