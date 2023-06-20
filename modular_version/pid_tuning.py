#!/usr/bin/env python3
import minimalmodbus
import RPi.GPIO as GPIO
import openpyxl
import datetime
import time
import sys

# Set up sensor 1: address=0x001
sensor1 = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
sensor1.serial.baudrate = 9600
sensor1.serial.timeout = 0.2
sensor1.mode = minimalmodbus.MODE_RTU

# GPIO setup
relayPin = 12
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(relayPin, GPIO.OUT)

# Create or open Excel workbook
excel_name = f"pengujian_tuning_PID_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
wb = openpyxl.Workbook()
ws = wb.active
ws.title = 'Data'

# Write headers
ws.append(['Timestamp', 'Temperature 1', 'Humidity 1', 'Humidity Target', 'Relay Status'])


# Read climate sensors
def read_climate_sensors():
    temperature1 = sensor1.read_register(1, 1)
    humidity1 = sensor1.read_register(0, 1)

    readings = {
        'sensor1': {
            'temperature': temperature1,
            'humidity': humidity1
        }
    }

    return readings

# Function to control the relay based on humidity threshold
def control_relay(humidity_threshold, readings):
    humidity = calibrate_humidity(readings['sensor1']['humidity'])
    relay_status = "active" if humidity <= humidity_threshold else "non active"

    if humidity <= humidity_threshold:
        GPIO.output(relayPin, GPIO.HIGH)
    else:
        GPIO.output(relayPin, GPIO.LOW)
    
    return relay_status

# Function to calibrate humidity value
def calibrate_humidity(value):
    if value <= 20:
        return value - 4
    elif 20 < value <= 60:
        return value - 3.5
    elif 60 < value <= 80:
        return value - 4
    elif 80 < value <= 100:
        return value - 4.5
    else:
        return value

# Function to log data in Excel
def log_data(readings, humidity_threshold):
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    sensor1_temp = readings['sensor1']['temperature']
    sensor1_hum = readings['sensor1']['humidity']
    
    calibrated_humidity = calibrate_humidity(sensor1_hum)

    # Write data
    ws.append([timestamp, sensor1_temp, calibrated_humidity, humidity_threshold, control_relay(humidity_threshold, readings)])

    # Save workbook
    wb.save(excel_name)

# Clean up GPIO
def clean_up():
    GPIO.cleanup()
    wb.close()

# Main program
def main():
    humidity_threshold = 77  # Set your desired humidity threshold here

    try:
        while True:
            readings = read_climate_sensors()
            log_data(readings, humidity_threshold)
            relay_status = control_relay(humidity_threshold, readings)

            calibrated_humidity = calibrate_humidity(readings['sensor1']['humidity'])

            # Display log in the monitor
            print("____________________________________")
            print("Current Sensor Readings: ({})".format(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
            print("        Temp 1:{:.3f} | Hum 1:{:.3f}".format(readings['sensor1']['temperature'], calibrated_humidity))
            print("Humidity Target: {} %".format(humidity_threshold))
            print("Relay Status: {}".format(relay_status))

            # Delay between readings
            time.sleep(1)
    except KeyboardInterrupt:
        clean_up()
        sys.exit(0)

if __name__ == '__main__':
    main()
