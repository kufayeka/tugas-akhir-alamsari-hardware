#!/usr/bin/env python3
import minimalmodbus
import RPi.GPIO as GPIO
import openpyxl
import datetime
import time
import sys
import threading

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
ws.append(['Timestamp', 'Temperature 1', 'Humidity 1', 'Relay Status'])

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

# Function to control the relay
def control_relay():
    GPIO.setmode(GPIO.BOARD)
    while True:
        GPIO.output(relayPin, GPIO.HIGH)
        time.sleep(5)
        GPIO.output(relayPin, GPIO.LOW)
        time.sleep(5)

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
def log_data(readings):
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    sensor1_temp = readings['sensor1']['temperature']
    sensor1_hum = readings['sensor1']['humidity']

    calibrated_humidity = calibrate_humidity(sensor1_hum)

    # Write data
    ws.append([timestamp, sensor1_temp, calibrated_humidity, "Active"])

    # Save workbook
    wb.save(excel_name)

# Clean up GPIO
def clean_up():
    GPIO.cleanup()
    wb.close()

# Main program
def main():
    try:
        control_relay_thread = threading.Thread(target=control_relay)
        control_relay_thread.start()

        while True:
            readings = read_climate_sensors()
            log_data(readings)

            calibrated_humidity = calibrate_humidity(readings['sensor1']['humidity'])

            # Display log in the monitor
            print("____________________________________")
            print("Current Sensor Readings: ({})".format(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
            print("        Temp 1:{:.3f} | Hum 1:{:.3f}".format(readings['sensor1']['temperature'], calibrated_humidity))
            relay_status = "Active" if GPIO.input(relayPin) == GPIO.HIGH else "Inactive"  # Read GPIO pin status
            print("Relay Status:", relay_status)

            # Delay between readings
            time.sleep(1)
    except KeyboardInterrupt:
        clean_up()
        sys.exit(0)

if __name__ == '__main__':
    main()
