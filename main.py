#!/usr/bin/env python3
import datetime
import global_variables as gv
import multiprocessing
import openpyxl
import RPi.GPIO as GPIO
import schedule
import time

from climate_sensors import read_climate_sensors
from mqtt import MQTTModuleClass
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

# mqtt configurations
broker_address = "192.168.43.3"
#broker_address = "broker.hivemq.com"
broker_username = "petra_mqtt_broker"
broker_password = "petraMqttBroker777"

# multiprocessing variables manager
manager = multiprocessing.Manager()

mqtt_instance = None

def connect_to_broker():
    try:
        instance = MQTTModuleClass(broker_address, broker_username, broker_password)
        instance.subscribe_topic("pussy_wet")
        return True, instance
    except Exception as e:
        print("Failed to connect to the MQTT broker:", str(e))
        connect_to_broker()
        time.sleep(3)
        return False, None

# map/scale PID output into 0 - 100 range
def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

def process1(gv):
    def sensors_job():
        # read RS485 climate sensors
        read_climate_sensors()
        timestamp = datetime.datetime.now().strftime('%H:%M:%S')  # Get the current timestamp
        print("____________________________________")
        print(f"Current Sensor Readings: ({timestamp})")
        print(f"\tTemp 1:{gv.temp1.get():.3f} | Hum 1:{gv.hum1.get():.3f}")
        print(f"\tTemp 2:{gv.temp2.get():.3f} | Hum 2:{gv.hum2.get():.3f}")
        print()
        print(f"PID Parameters: kp:{gv.PID_kp.get()} | ki:{gv.PID_ki.get()} | kd:{gv.PID_kd.get()}")
        print("PID PV:", gv.PID_pv.get())
        print("PID Set Point:", gv.PID_set_point.get())
        print(f"PID Output: {gv.PID_output.get():.3f}")
        print()
        print("PWM Timing / DutyCycle (secs): ")
        print(f"\tHigh Time:{gv.PWM_high_time.get():.3f} | Low Time:{gv.PWM_low_time.get():.3f}")

    while True:
        sensors_job()
        time.sleep(1)

def process2(gv):
    while True:

        # calculate PID out based on feedback value (humidity/temperature)
        feedback = gv.temp1.get()
        set_point = gv.PID_set_point.get()

        pid.setpoint = set_point
        pid_out = pid(feedback)

        # update tuning with the newest setting while system is running
        pid.tunings = (gv.PID_kp.get(), gv.PID_ki.get(), gv.PID_kd.get())
        p, i, d = pid.components

        # set the pv variable based on hum1 value
        gv.PID_pv.set(round(feedback, 3))
        gv.PID_output.set(round(pid_out, 3))

        # convert PID output to GPIO high and low time range (relayPin)
        # maximum interval ON = 10 seconds
        max_interval = gv.PWM_max_interval.get()
        min_interval = gv.PWM_min_interval.get()
        swap_intervals = False

        if swap_intervals:
            min_interval, max_interval = max_interval, min_interval

        high_time = map_range(pid_out, 0, 100, min_interval, max_interval)
        low_time = map_range(pid_out, 0, 100, max_interval, min_interval)

        # set the PWM timing variable based on map calculation value
        gv.PWM_high_time.set(high_time)
        gv.PWM_low_time.set(low_time)

        # guard PWM
        if low_time < 0:
            gv.PWM_enabled.set(0)
        else:
            gv.PWM_enabled.set(1)

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
    mqtt_is_connected, mqtt_instance = connect_to_broker()

    while True:
        if mqtt_is_connected:
            mqtt_instance.publish_message("pussy_wet", "and_moist")
            time.sleep(2)

#############################################################################
if __name__ == '__main__':

    p1 = multiprocessing.Process(target=process1, args=(gv,))
    p2 = multiprocessing.Process(target=process2, args=(gv,))
    p3 = multiprocessing.Process(target=process3, args=(gv.PWM_enabled, gv.PWM_high_time, gv.PWM_low_time,))
    p4 = multiprocessing.Process(target=process4, args=())

    try:
        print("STARTING...")
        GPIO.output(relayPin, GPIO.LOW)
        p1.start()
        p2.start()
        p3.start()
        p4.start()

        p1.join()
        p2.join()
        p3.join()
        p4.join()

    except KeyboardInterrupt:
        GPIO.cleanup()
        p1.terminate()
        p2.terminate()
        p3.terminate()
        p4.terminate()

    finally:
        GPIO.cleanup()  # this ensures a clean exit

        print("TERMINATED...")
        time.sleep(0.1)