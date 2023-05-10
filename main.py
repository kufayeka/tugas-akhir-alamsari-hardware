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
pid = PID(0.5, 0.05, 0.01)
pid.output_limits = (0, 100)
pid.sample_time = 0.001
pid.auto_mode = True
pid.proportional_on_measurement = False

# multiprocessing variables manager
manager = multiprocessing.Manager()

# map/scale PID output into 0 - 100 range
def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

# Method process1 is used to read RS485 climate sensors, calculate PID output value based on the climate sensors data
def process1(temp1, temp2, hum1, hum2, PID_kp, PID_ki, PID_kd, PID_output, PID_set_point, PID_pv, PWM_high_time, PWM_low_time):
    while True:
        # read RS485 climate sensors
        read_climate_sensors()

        print("____________________________________")
        print("Current Sensor Readings: ")
        print(f"\tTemp1:{temp1.value:.3f} | Hum1:{hum1.value:.3f}")
        print(f"\tTemp2:{temp2.value:.3f} | Hum2:{hum2.value:.3f}")
        print()
        print(f"PID Parameters: kp:{PID_kp.value} | ki:{PID_ki.value} | kd:{PID_kd.value}")
        print("PID PV:", PID_pv.value)
        print("PID Set Point:", PID_set_point.value)
        print("PID Output:", PID_output.value)
        print()
        print("PWM Timing / DutyCycle (secs): ")
        print(f"\tHigh Time:{PWM_high_time.value} | Low Time:{PWM_low_time.value}")

        time.sleep(1)

def process2(hum1, PID_kp, PID_ki, PID_kd, PID_pv, PID_output, PID_set_point, PWM_high_time, PWM_low_time):
    while True:
        # calculate PID out based on feedback value (humidity/temperature)
        PID_kp, PID_ki, PID_kd = pid.components
        feedback = hum1.value
        pid.setpoint = PID_set_point.value
        pid_out = pid(feedback)

        # set the pv variable based on hum1 value
        PID_pv.value = feedback
        PID_output.value = pid_out

        # convert PID output to GPIO high and low time range (relayPin)
        # maximum interval ON = 10 seconds
        max_interval = 5
        high_time = map_range(pid_out, 0, 100, 0, max_interval)
        low_time = map_range(pid_out, 0, 100, max_interval, 0)

        # set the PWM timing variable based on map calculation value
        PWM_high_time.value = high_time
        PWM_low_time.value = low_time

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
    p1 = multiprocessing.Process(target=process1, args=(gv.temp1, gv.temp2, gv.hum1, gv.hum2, gv.PID_kp, gv.PID_ki, gv.PID_kd, gv.PID_output, gv.PID_set_point, gv.PID_pv, gv.PWM_high_time, gv.PWM_low_time))
    p2 = multiprocessing.Process(target=process2, args=(gv.hum1, gv.PID_kp, gv.PID_ki, gv.PID_kd, gv.PID_pv, gv.PID_output, gv.PID_set_point, gv.PWM_high_time, gv.PWM_low_time))
    p3 = multiprocessing.Process(target=process3, args=(gv.pid_output, gv.PID_ki, gv.PID_kd, gv.PID_pv, gv.sensors_climate_readings,))

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