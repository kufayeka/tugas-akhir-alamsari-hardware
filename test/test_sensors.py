#!/usr/bin/env python3
import datetime
from time import sleep
import minimalmodbus

# set up sensor 1 : address=0x001
sensor1 = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
sensor1.serial.baudrate = 9600
sensor1.serial.timeout = 0.2
sensor1.mode = minimalmodbus.MODE_RTU

# set up sensor 2 : address=0x002
sensor2 = minimalmodbus.Instrument('/dev/ttyUSB0', 2)
sensor2.serial.baudrate = 9600
sensor2.serial.timeout = 0.2
sensor2.mode = minimalmodbus.MODE_RTU

def calibrate_humidity(value):
    if value <= 20:
        return value - 4
    elif 20 < value <= 60:
        return value - 3.5
    elif 60 < value <= 80:
        return value - 3.8
    elif 80 < value <= 100:
        return value - 4.5
    else:
        return value

def read_climate_sensors():

    # Membaca nilai sensor1
    temperature1 = sensor1.read_register(1, 1)  
    humidity1 = sensor1.read_register(0, 1) 

    # Membaca nilai sensor2
    temperature2 = sensor2.read_register(1, 1)  
    humidity2 = sensor2.read_register(0, 1) 

    # Print sensor readings
    calibrated_hum1_value = calibrate_humidity(humidity1)
    calibrated_hum2_value = calibrate_humidity(humidity2)

    print("____________________________________")
    print(f"Current Sensor Readings:)")
    print(f"\tTemp 1: {temperature1:.3f}°C | Hum 1: {calibrated_hum1_value:.3f}%")
    print(f"\tTemp 2: {temperature2:.3f}°C | Hum 2: {calibrated_hum2_value:.3f}%")

try:  
    # here you put your main loop or block of code  
    while True:  
        read_climate_sensors()
        sleep(1) 
  
except KeyboardInterrupt:  
    # here you put any code you want to run before the program   
    # exits when you press CTRL+C  
    print("interuptted")
  
except:  
    # this catches ALL other exceptions including errors.  
    # You won't get any error messages for debugging  
    # so only use it once your code is working  
    print("error")
  
finally:  
    # this ensures a clean exit
    print("finally")