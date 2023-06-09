import datetime
import time
import threading
import multiprocessing
import json
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from utils import climate_sensors
from utils import pid_calculator

sensor1_humidity = 0
sensors_readings = 0
PID_kp, PID_ki, PID_kd = 0.5, 0.05, 0.01
PID_set_point = 80
PID_output = 0
PWM_enabled = False
PWM_high_time = 1   # Dynamic interval for relay_on()
PWM_low_time = 2    # Dynamic interval for relay_off()
min_interval, max_interval, swap_intervals = 0, 5, False

# Informasi broker MQTT
broker_address = "203.189.122.131"
broker_username = "petra_mqtt_broker"
broker_password = "petraMqttBroker777"
clean_session = False
clientID = "hffsfusjdfksnfjsbfweoijdlkm"
subscribe_qos = 1

# MQTT Topic
topikDataRealtime = "petra/alamsari/kumbung_jamur/real_time/climate"
topikDataLogger = "petra/alamsari/kumbung_jamur/data_logger/climate"
topikSettingParameter = "petra/alamsari/kumbung_jamur/setting/all_parameters"

def command_line_printer(ts):
    global sensors_readings 
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

def relay_on(ts):
    print("RELAY ON", ts)

def relay_off(ts):
    print("RELAY OFF", ts)

def sensor_work():
    sensor_interval = 1  # 1 second interval for read_sensor()
    data_interval = 10   # 10 second interval for record_data()
    logger_interval = 3   # 10 second interval for record_data()
    sensor_last_time = time.time()
    data_last_time = time.time()
    data_logger_last_time = time.time()
    global PWM_high_time, PWM_low_time, PWM_enabled, PID_output

    # Fungsi yang dipanggil saat koneksi berhasil dibuat
    def on_connect(client, userdata, flags, rc):
        print("Terhubung ke broker")
        client.subscribe(topikDataLogger, qos=subscribe_qos)
        client.subscribe(topikDataRealtime, qos=subscribe_qos)
        client.subscribe(topikSettingParameter, qos=subscribe_qos)

    # Fungsi yang dipanggil saat menerima pesan MQTT
    def on_message(client, userdata, msg):
        print(f"Pesan diterima: {msg.topic} {msg.payload.decode()}")
        #PID_kp, PID_ki, PID_kd = 0.5, 0.05, 0.01
        #PID_set_point = 80

    # Inisialisasi client MQTT
    client = mqtt.Client(clientID, clean_session=clean_session)

    # Set username dan password jika ada
    if broker_username and broker_password:
        client.username_pw_set(broker_username, broker_password)

    # Menetapkan fungsi yang akan dipanggil saat terhubung dan menerima pesan
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_address)

    client.loop_start()

    prev_payload = None

    while True:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Get timestamp with milliseconds (removing the last 3 digits)

        # Read the sensor data every 1 second
        if time.time() - sensor_last_time >= sensor_interval:
            command_line_printer(timestamp)
            sensor1_temp = sensors_readings['sensor1']['temperature']
            sensor1_hum = sensors_readings['sensor1']['humidity']
            sensor2_temp = sensors_readings['sensor2']['temperature']
            sensor2_hum = sensors_readings['sensor2']['humidity']

            payload = {
                'sensor1': {
                    'temperature': sensor1_temp,
                    'humidity': sensor1_hum,
                },
                'sensor2': {
                    'temperature': sensor2_temp,
                    'humidity': sensor2_hum,
                }
            }

            if payload != prev_payload:
                client.publish(topikDataRealtime, str(payload), qos=1)
                prev_payload = payload

            sensor_last_time = time.time()
        
        # Publish to data logger
        if time.time() - data_logger_last_time >= logger_interval:
            sensor1_temp = sensors_readings['sensor1']['temperature']
            sensor1_hum = sensors_readings['sensor1']['humidity']
            sensor2_temp = sensors_readings['sensor2']['temperature']
            sensor2_hum = sensors_readings['sensor2']['humidity']

            payload = {
                'sensor1': {
                    'temperature': sensor1_temp,
                    'humidity': sensor1_hum,
                    'timestamp': timestamp
                },
                'sensor2': {
                    'temperature': sensor2_temp,
                    'humidity': sensor2_hum,
                    'timestamp': timestamp
                }
            }
            
            client.publish(topikDataLogger, str(payload), qos=1)
            data_logger_last_time = time.time()

        # Continuously calculate PID
        PWM_high_time, PWM_low_time, PWM_enabled, PID_output = pid_calculator.calculate_pid_output(PID_kp, PID_ki, PID_kd, PID_set_point, sensor1_humidity, min_interval, max_interval, swap_intervals)
        
        time.sleep(0.1)  # Add a small delay to reduce CPU usage

def relay_work():
    while True:
        timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]  # Get timestamp with milliseconds (removing the last 3 digits)

        # Turn on the relay
        #relay_on(timestamp)
        GPIO.output(relayPin, GPIO.LOW)
        GPIO.output(fanPin, GPIO.HIGH)
        time.sleep(1)

        # Turn off the relay
        #relay_off(timestamp)
        #time.sleep(2)

if __name__ == '__main__':
    sensor_process = multiprocessing.Process(target=sensor_work)
    relay_process = multiprocessing.Process(target=relay_work)

    # setup GPIO
    relayPin = 12
    fanPin = 18	
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(relayPin, GPIO.OUT)
    GPIO.setup(fanPin, GPIO.OUT)

    try:
        print("STARTING...\n\n")
        time.sleep(1)

        sensor_process.start()
        relay_process.start()

        sensor_process.join()
        relay_process.join()

    except KeyboardInterrupt:
        print("\n\nKeyboard Interrupt Detected...")

    finally:
        sensor_process.kill()
        relay_process.kill()

        GPIO.cleanup()
        print("gpio cleaned up!")
        print("excel data saved!")
        print("TERMINATED!")
