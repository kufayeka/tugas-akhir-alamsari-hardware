#!/usr/bin/env python3
import json
import time
import global_variables as gv

import minimalmodbus
import multiprocessing
import requests
import RPi.GPIO as GPIO

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

def read_climate_sensors():

    # Membaca nilai sensor1
    temperature1 = sensor1.read_register(1, 1)  
    humidity1 = sensor1.read_register(0, 1) 

    # Membaca nilai sensor2
    temperature2 = sensor2.read_register(1, 1)  
    humidity2 = sensor2.read_register(0, 1) 

    # Assign nilai variabel
    gv.temp1.value = temperature1
    gv.hum1.value = humidity1
    gv.temp2.value = temperature2
    gv.hum2.value = humidity2
