import datetime
import time
import threading
import multiprocessing 
import json
import openpyxl
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from utils import climate_sensors
from utils import pid_calculator

sensor1_humidity = 0
sensors_readings = 0
PID_kp, PID_ki, PID_kd = 10, 0.1, 1
PID_set_point = 80
PID_output = 0
PWM_enabled = False
PWM_high_time = 1   # Dynamic interval for relay_on()
PWM_low_time = 2    # Dynamic interval for relay_off()
interval_time_from, interval_time_to, swap_intervals = 0, 5, False

sensor_reading_interval = 1  # 1 second interval for read_sensor()
interval_data_logger = 10

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

manager = multiprocessing.Manager()

pwm_en = manager.Value('d', 0.000)
pwm_high = manager.Value('d', 0.000)
pwm_low = manager.Value('d', 0.000)

excel_name = f"pengujian_kontroler_PID_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
wb = openpyxl.Workbook()
ws = wb.active
ws.title = 'Data'
ws.append(['Timestamp', 'Temperature 1', 'Temperature 2', 'Humidity 1', 'Humidity 2', 'Set Point'])


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
    print(f"Data Logger Interval: {interval_data_logger}")
    print(f"Interval Time (from, to): ({interval_time_from}, {interval_time_to})")
    print()
    print("PWM Enabled:", PWM_enabled)
    print("PWM Timing / DutyCycle (secs): ")
    print(f"\tHigh Time:{PWM_high_time:.3f} | Low Time:{PWM_low_time:.3f}")

def relay_on(ts):
    print("RELAY ON", ts)

def relay_off(ts):
    print("RELAY OFF", ts)

def sensor_work():
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
        #print(f"Pesan diterima: {msg.topic} {msg.payload.decode()}")
        print("")

        global PID_kp
        global PID_ki
        global PID_kd
        global PID_set_point
        global interval_time_from 
        global interval_time_to
        global interval_data_logger

        #print(f"Pesan diterima: {msg.topic} {msg.payload.decode()}")

        if(str(msg.topic) == topikSettingParameter):
            global PID_kp
            global PID_ki
            global PID_kd
            global PID_set_point
            global interval_time_from 
            global interval_time_to
            global interval_data_logger

            payload = json.loads(msg.payload.decode())
            pid_settings = payload.get("pid_settings")
            data_logger_settings = payload.get("data_logger_settings")

            if pid_settings:
                PID_set_point = float(pid_settings.get("set_point"))
                PID_kp = float(pid_settings.get("kp"))
                PID_ki = float(pid_settings.get("ki"))
                PID_kd = float(pid_settings.get("kd"))
                interval_time_from = float(pid_settings.get("interval_from"))
                interval_time_to = float(pid_settings.get("interval_to"))

            if data_logger_settings:
                interval_data_logger = float(data_logger_settings.get("interval"))

            print(f"Setting Parameter PID/Data Logger Baru Diterima!")

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

    sensor1_temp = None
    sensor1_hum = None
    sensor2_temp = None
    sensor2_hum = None

    while True:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Get timestamp with milliseconds (removing the last 3 digits)

        # Read the sensor data every 1 second
        if time.time() - sensor_last_time >= sensor_reading_interval:
            command_line_printer(timestamp)
            new_sensor1_temp = sensors_readings['sensor1']['temperature']
            new_sensor1_hum = sensors_readings['sensor1']['humidity']
            new_sensor2_temp = sensors_readings['sensor2']['temperature']
            new_sensor2_hum = sensors_readings['sensor2']['humidity']

            if new_sensor1_temp != sensor1_temp or new_sensor1_hum != sensor1_hum or new_sensor2_temp != sensor2_temp or new_sensor2_hum != sensor2_hum:
                sensor1_temp = new_sensor1_temp
                sensor1_hum = new_sensor1_hum
                sensor2_temp = new_sensor2_temp
                sensor2_hum = new_sensor2_hum

                payload = {
                    'sensor1': {
                        'temperature': sensor1_temp,
                        'humidity': sensor1_hum,
                    },
                    'sensor2': {
                        'temperature': sensor2_temp,
                        'humidity': sensor2_hum,
                    },
                    'timestamp': timestamp
                }

                client.publish(topikDataRealtime, json.dumps(payload), qos=1)

            sensor_last_time = time.time()
                
        # Publish to data logger
        if time.time() - data_logger_last_time >= interval_data_logger:
            sensor1_temp = sensors_readings['sensor1']['temperature']
            sensor1_hum = sensors_readings['sensor1']['humidity']
            sensor2_temp = sensors_readings['sensor2']['temperature']
            sensor2_hum = sensors_readings['sensor2']['humidity']

            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # excel recorder
            ws.append([timestamp, sensor1_temp, sensor2_temp, sensor1_hum, sensor2_hum, PID_set_point])
            wb.save(excel_name)

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
            #print("Rekam Data Klimat:", timestamp)
            data_logger_last_time = time.time()

        # Continuously calculate PID
        PWM_high_time, PWM_low_time, PWM_enabled, PID_output = pid_calculator.calculate_pid_output(PID_kp, PID_ki, PID_kd, PID_set_point, sensor1_humidity, interval_time_from, interval_time_to, swap_intervals)
        pwm_en.value = PWM_enabled
        pwm_high.value = PWM_high_time
        pwm_low.value = PWM_low_time
        time.sleep(0.1)  # Add a small delay to reduce CPU usage

def relay_work(pwm_en, pwm_high, pwm_low):
    while True:
        if pwm_en.value == True:
            # Turn on the relay
            #print("PWM Enabled", pwm_high.value, pwm_low.value)
            GPIO.output(relayPin, GPIO.LOW)
            time.sleep(pwm_high.value)
            # Turn off the relay
            GPIO.output(relayPin, GPIO.LOW)
            time.sleep(pwm_low.value)
        else:
            GPIO.output(relayPin, GPIO.LOW)
            #print("PWM Disabled", pwm_high.value, pwm_low.value)
            time.sleep(1)

if __name__ == '__main__':
    pwm_dict = manager.dict()
    sensor_process = multiprocessing.Process(target=sensor_work,)
    relay_process = multiprocessing.Process(target=relay_work, args=(pwm_en, pwm_high, pwm_low,))

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
        wb.close()

    finally:
        sensor_process.kill()
        relay_process.kill()

        GPIO.cleanup()
        print("gpio cleaned up!")
        print("excel data saved!")
        print("TERMINATED!")
