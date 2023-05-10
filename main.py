#!/usr/bin/env python3
import global_variables as gv
import multiprocessing
import RPi.GPIO as GPIO
import time

from climate_sensors import read_climate_sensors
from simple_pid import PID

relayPin = 12
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(relayPin, GPIO.OUT)

# setup pid
pid = PID(gv.pid_parameters['kp'], gv.pid_parameters['ki'], gv.pid_parameters['kp'])
pid.output_limits = (0, 100)
pid.sample_time = 0.01
pid.auto_mode = True
pid.proportional_on_measurement = False

# map/scale PID output into 0 - 100 range
def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

# Method process1 is used to read RS485 climate sensors, calculate PID output value based on the climate sensors data
def process1(temp1, temp2, hum1, hum2):
    while True:
        # read RS485 climate sensors
        read_climate_sensors()

        print("____________________________________")
        print("Current Sensor Readings: ")
        print(f"\tTemp1:{temp1.value:.3f} | Hum1:{hum1.value:.3f}")
        print(f"\tTemp2:{temp2.value:.3f} | Hum2:{hum2.value:.3f}")
        time.sleep(1)

def process2(pid_output, pid_parameters, sensors_climate_readings):
    while True:
        # calculate PID out based on feedback value (humidity/temperature)
        feedback = sensors_climate_readings['temp1']
        pid.setpoint = pid_parameters['set_point']
        pid_out = pid(feedback)

        # convert PID output to GPIO high and low time range (relayPin)
        # maximum interval ON = 10 seconds
        max_interval = 5
        pid_output['high_time'] = map_range(pid_out, 0, 100, 0, max_interval)
        pid_output['low_time'] = map_range(pid_out, 0, 100, max_interval, 0)

        time.sleep(1)

def process3(pid_output, pid_parameters, sensors_climate_readings):
    while True:
        if pid_output['pwm_enabled'] == True:
            GPIO.output(relayPin, GPIO.HIGH) 
            time.sleep(pid_output['high_time']) 
            if pid_output['low_time'] != 0:
                GPIO.output(relayPin, GPIO.LOW) 
                time.sleep(pid_output['low_time']) 
        else: 
            GPIO.output(relayPin, GPIO.LOW) 
            time.sleep(1)

if __name__ == '__main__':
    p1 = multiprocessing.Process(target=process1, args=(gv.temp1, gv.temp2, gv.hum1, gv.hum2,))
    p2 = multiprocessing.Process(target=process2, args=(gv.pid_output, gv.pid_parameters, gv.sensors_climate_readings,))
    p3 = multiprocessing.Process(target=process3, args=(gv.pid_output, gv.pid_parameters, gv.sensors_climate_readings,))

    try:
        GPIO.output(relayPin, GPIO.LOW)
        print("STARTING...")
        p1.start()
        p2.start()
        #p3.start()

        p1.join()
        p2.join()
        #p3.join()
            
    except KeyboardInterrupt:
        GPIO.cleanup()
        p1.terminate()
        p2.terminate()
        #p3.terminate()
        print("TERMINATED...")
        time.sleep(0.1)
    
    except ValueError:  
        print("PROGRAM ERROR")
        time.sleep(0.5)

    finally:  
        GPIO.cleanup() # this ensures a clean exit
        time.sleep(0.5)