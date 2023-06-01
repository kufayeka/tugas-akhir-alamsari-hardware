#!/usr/bin/env python3
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
        return value - 4
    elif 80 < value <= 100:
        return value - 4.5
    else:
        return value

def read_climate_sensors():
    # Membaca nilai sensor1
    temperature1 = sensor1.read_register(1, 1)
    humidity1 = sensor1.read_register(0, 1)
    calibrated_humidity1 = calibrate_humidity(humidity1)

    # Membaca nilai sensor2
    temperature2 = sensor2.read_register(1, 1)
    humidity2 = sensor2.read_register(0, 1)
    calibrated_humidity2 = calibrate_humidity(humidity2)

    # Mengembalikan hasil pembacaan dalam format dictionary
    readings = {
        'sensor1': {
            'temperature': temperature1,
            'humidity': calibrated_humidity1
        },
        'sensor2': {
            'temperature': temperature2,
            'humidity': calibrated_humidity2
        }
    }

    return readings
