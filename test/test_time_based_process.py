import datetime
import time

sensor_interval = 1     # 1 second interval for read_sensor()
data_interval = 10      # 10 second interval for record_data()
relay_on_time = 10      # Dynamic interval for relay_on()
relay_off_time = 2      # Dynamic interval for relay_off()

sensor_last_time = time.time()
data_last_time = time.time()
relay_last_time = time.time()

while True:
    timestamp = datetime.datetime.now().strftime('%H:%M:%S')
    # Read the sensor data every 1 second
    if time.time() - sensor_last_time >= sensor_interval:
        print("read sensor", timestamp)
        sensor_last_time = time.time()

    # Record the data every 10 seconds
    if time.time() - data_last_time >= data_interval:
        print("record sensor data...", timestamp)
        data_last_time = time.time()

    # Trigger relay on with dynamic interval
    if time.time() - relay_last_time >= relay_on_time:
        print("RELAY ON", timestamp)
        relay_last_time = time.time()

    # Sleep for the relay off time
    if time.time() - relay_last_time >= relay_off_time:
        print("RELAY OFF", timestamp)
        relay_last_time = time.time()

    # Continuously calculate PID
    #calculate_pid()
