#!/usr/bin/env python3
import datetime
import global_variables as gv
import multiprocessing
import openpyxl
import RPi.GPIO as GPIO
import time

from climate_sensors import read_climate_sensors
from simple_pid import PID

# setup GPIO
relayPin = 12
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(relayPin, GPIO.OUT)

# setup pid
pid = PID(0.5, 0.05, 0.01)
pid.output_limits = (0, 100)
pid.sample_time = 0.01
pid.auto_mode = True
pid.proportional_on_measurement = False
pid.differential_on_measurement = True


# multiprocessing variables manager
manager = multiprocessing.Manager()

# excel (data testing)
excel_name = f"data_klimat_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"

wb = openpyxl.Workbook()
ws = wb.active
ws.title = 'Data'
ws.append(['Set Point', 'Output', 'Timestamp'])

ws2 = wb.create_sheet('Parameters')
ws2.append(['Kp', 'Ki', 'Kd'])
ws2.append([gv.PID_kp.get(), gv.PID_ki.get(), gv.PID_kd.get()])

# map/scale PID output into 0 - 100 range
def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

# Method process1 is used to read RS485 climate sensors, calculate PID output value based on the climate sensors data
def process1(temp1, temp2, hum1, hum2, PID_kp, PID_ki, PID_kd, PID_output, PID_set_point, PID_pv, PWM_high_time, PWM_low_time):
    # Initialize the previous temperature value
    prev_temp_value = None

    while True:
        # read RS485 climate sensors
        read_climate_sensors()

        timestamp = datetime.datetime.now().strftime('%H:%M:%S') # Get the current timestamp
        print("____________________________________")
        print(f"Current Sensor Readings: ({timestamp})")
        print(f"\tTemp 1:{temp1.get():.3f} | Hum 1:{hum1.get():.3f}")
        print(f"\tTemp 2:{temp2.get():.3f} | Hum 2:{hum2.get():.3f}")
        print()
        print(f"PID Parameters: kp:{PID_kp.get()} | ki:{PID_ki.get()} | kd:{PID_kd.get()}")
        print("PID PV:", PID_pv.get())
        print("PID Set Point:", PID_set_point.get())
        print(f"PID Output: {PID_output.get():.3f}")
        print()
        print("PWM Timing / DutyCycle (secs): ")
        print(f"\tHigh Time:{PWM_high_time.get():.3f} | Low Time:{PWM_low_time.get():.3f}")

        # record the climate data only when changes detected
        #current_temp_value = temp1.value
        #if current_temp_value != prev_temp_value:
        #    row = (current_temp_value, timestamp)
        #    ws.append(row)
        #    wb.save(excel_name)
        #    prev_temp_value = current_temp_value

        # record the climate data every second
        #row = (PID_set_point.value, PID_pv.value, timestamp)
        #ws.append(row)
        #wb.save(excel_name)

        time.sleep(10)

def process2(hum1, PID_kp, PID_ki, PID_kd, PID_pv, PID_output, PID_set_point, PWM_enabled, PWM_max_interval, PWM_min_interval, PWM_high_time, PWM_low_time):
    while True:

        # calculate PID out based on feedback value (humidity/temperature)
        feedback = hum1.get()
        set_point = PID_set_point.get()

        pid.setpoint = set_point
        pid_out = pid(feedback)

        # update tuning with the newest setting while system is running
        pid.tunings = (PID_kp.get(), PID_ki.get(), PID_kd.get())
        p, i, d = pid.components

        # set the pv variable based on hum1 value
        PID_pv.set(round(feedback, 3))
        PID_output.set(round(pid_out, 3))

        # convert PID output to GPIO high and low time range (relayPin)
        # maximum interval ON = 10 seconds
        max_interval = PWM_max_interval.value
        min_interval = PWM_min_interval.value
        swap_intervals = False

        if swap_intervals:
            min_interval, max_interval = max_interval, min_interval

        high_time = map_range(pid_out, 0, 100, min_interval, max_interval)
        low_time = map_range(pid_out, 0, 100, max_interval, min_interval)

        # set the PWM timing variable based on map calculation value
        PWM_high_time.set(high_time)
        PWM_low_time.set(low_time)

        # guard PWM
        if low_time < 0:
            PWM_enabled.set(0)
        else:
            PWM_enabled.set(1)

def process3(PWM_enabled, PWM_high_time, PWM_low_time):
    while True:
        if PWM_enabled.value == 1:
            # set the relayPin to HIGH as long as the PWM_high_time.value
            GPIO.output(relayPin, GPIO.LOW) 
            time.sleep(PWM_high_time.get()) 
            # if PWM_low_time.value != 0, set the relayPin to LOW as long as the PWM_high_time.value
            if PWM_low_time.value != 0:
                GPIO.output(relayPin, GPIO.LOW) 
                time.sleep(PWM_low_time.get()) 
        # if PWM_enabled.value == 0, set relayPin to LOW
        else: 
            GPIO.output(relayPin, GPIO.LOW) 
            time.sleep(1)

def process4():
    while True:
        print("process4 running")
        time.sleep(1)

def process5():
    while True:
        print("process5 running")
        time.sleep(3)

#############################################################################
    
if __name__ == '__main__':
    p1 = multiprocessing.Process(target=process1, args=(gv.temp1, gv.temp2, gv.hum1, gv.hum2, gv.PID_kp, gv.PID_ki, gv.PID_kd, gv.PID_output, gv.PID_set_point, gv.PID_pv, gv.PWM_high_time, gv.PWM_low_time,))
    p2 = multiprocessing.Process(target=process2, args=(gv.temp1, gv.PID_kp, gv.PID_ki, gv.PID_kd, gv.PID_pv, gv.PID_output, gv.PID_set_point, gv.PWM_enabled, gv.PWM_max_interval, gv.PWM_min_interval, gv.PWM_high_time, gv.PWM_low_time,))
    p3 = multiprocessing.Process(target=process3, args=(gv.PWM_enabled, gv.PWM_high_time, gv.PWM_low_time,))
    p4 = multiprocessing.Process(target=process4, args=())
    p5 = multiprocessing.Process(target=process5, args=())

    try:
        print("STARTING...")
        GPIO.output(relayPin, GPIO.LOW)
        p1.start()
        p2.start()
        p3.start()
        #p4.start()
        #p5.start()

        p1.join()
        p2.join()
        p3.join()
        #p4.join()
        #p5.join()
            
    except KeyboardInterrupt:
        GPIO.cleanup()
        p1.terminate()
        p2.terminate()
        p3.terminate()
        #p4.terminate()
        #p5.terminate()

        # Save the Excel workbook & Close the workbook when done
        #wb.save(excel_name)
        wb.close()

        print("TERMINATED...")
        time.sleep(0.1)
    
    except ValueError:  
        print("PROGRAM ERROR")
        time.sleep(0.5)

    finally:  
        GPIO.cleanup() # this ensures a clean exit
        time.sleep(0.5)