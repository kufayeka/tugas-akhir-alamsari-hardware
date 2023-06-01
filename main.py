#!/usr/bin/env python3
import datetime
import global_variables
import json
import multiprocessing
import openpyxl
import paho.mqtt.client as paho
from paho import mqtt
import RPi.GPIO as GPIO
import schedule
import time
import os
import sys

from climate_sensors import read_climate_sensors
from simple_pid import PID

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
ws2.append([global_variables.PID_kp.value, global_variables.PID_ki.value, global_variables.PID_kd.value])

def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

def restart_program():
    python = sys.executable
    os.execl(python, python, *sys.argv)

#############################################################################
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
        print("PWM Enabled:", gv.PWM_enabled.value)
        print("PWM Timing / DutyCycle (secs): ")
        print(f"\tHigh Time:{gv.PWM_high_time.get():.3f} | Low Time:{gv.PWM_low_time.get():.3f}")
        print()
    
    while True:
        sensors_job()

        # Record the climate data every 10 seconds
        if datetime.datetime.now().second % 10 == 0:
            timestamp = datetime.datetime.now().strftime('%H:%M:%S') # Get the current timestamp
            row = (gv.PID_set_point.value, gv.PID_pv.value, timestamp)
            ws.append(row)
            wb.save(excel_name)
        
        time.sleep(1)

def process2(gv):
    # setup pid
    pid = PID(0.5, 0.05, 0.01)
    pid.output_limits = (0, 100)
    pid.sample_time = 0.01
    pid.auto_mode = True
    pid.proportional_on_measurement = False
    pid.differential_on_measurement = True

    while True:
        # calculate PID out based on feedback value (humidity/temperature)
        feedback = gv.hum1.get()
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
        if high_time == 0:
            gv.PWM_enabled.value = 0
        else:
            gv.PWM_enabled.value = 1

def process3(gv, relayPin, fanPin):
    while True:
        if gv.PWM_enabled.value == 1:
            # set the relayPin to HIGH as long as the PWM_high_time.value
            GPIO.output(relayPin, GPIO.HIGH)
            GPIO.output(fanPin, GPIO.HIGH)
            #print("motor ON", gv.PWM_high_time.value, gv.PWM_enabled.value)
            time.sleep(gv.PWM_high_time.value)
            # if PWM_low_time.value != 0, set the relayPin to LOW as long as the PWM_high_time.value
            if gv.PWM_low_time.value != 0:
                GPIO.output(relayPin, GPIO.LOW)
                GPIO.output(fanPin, GPIO.LOW)
                #print("motor OFF", gv.PWM_low_time.value, gv.PWM_enabled.value)
                time.sleep(gv.PWM_low_time.value)
        # if PWM_enabled.value == 0, set relayPin to LOW
        else:
            GPIO.output(relayPin, GPIO.LOW)
            GPIO.output(fanPin, GPIO.LOW)
            time.sleep(1)

def process4(gv):
    def record_climate_sensors_results():
        temp1 = gv.temp1.get()
        temp2 = gv.temp2.get()
        hum1 = gv.hum1.get()
        hum2 = gv.hum2.get()
        payload = {
            'temp1': temp1,
            'temp2': temp2,
            'hum1': hum1,
            'hum2': hum2
        }
        publish_message("petra/alamsari/kumbung_jamur/data_logger_klimat", json.dumps(payload), 1, False)

    #schedule.every(1).second.do(record_climate_sensors_results)

    while True:
        #schedule.run_pending()
        record_climate_sensors_results()
        time.sleep(1)

is_connected = False  # Initial connection status is False
CLEAN_SESSION = True
KEEP_ALIVE_INTERVAL = 60 
#############################################################################
if __name__ == '__main__':
    gv = global_variables

    # setup GPIO
    relayPin = 12
    fanPin = 18	
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(relayPin, GPIO.OUT)
    GPIO.setup(fanPin, GPIO.OUT)
    
    broker_address = "203.189.122.131"
    broker_username = "petra_mqtt_broker"
    broker_password = "petraMqttBroker777"

    def on_log(client, userdata, level, buf):
        print("log:", buf)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            # Connection is established
            print()
            print("Connected to MQTT broker with result code " + str(rc))
            print()
        else:
            print("Failed to connect to Broker, return code =", rc)
            # Connection failed

    def on_disconnect(client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection from MQTT broker")
            # Disconnected from broker

    def on_message(client, userdata, msg):
        print("Received message: " + msg.topic + " " + str(msg.payload))

    def publish_message(topic, message, qos, retain):
        #print("Publishing:", client.is_connected(), topic, message, qos, retain)
        try:
            client.publish(topic, message, qos, retain)
        except:
            print("Not connected to MQTT broker. Cannot publish message.", client.is_connected())

    def subscribe_topic(topic):
        client.subscribe(topic, 1)

    def stop():
        client.loop_stop()
    
    def brrr():
        print("brrrrrr")

    client = paho.Client()
    client.username_pw_set(broker_username, broker_password)
    client.enable_logger()
    client.reconnect_delay_set(min_delay=1, max_delay=120)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.keep_alive = KEEP_ALIVE_INTERVAL
    
    p1 = multiprocessing.Process(target=process1, args=(gv,))
    p2 = multiprocessing.Process(target=process2, args=(gv,))
    p3 = multiprocessing.Process(target=process3, args=(gv, relayPin, fanPin))
    p4 = multiprocessing.Process(target=process4, args=(gv,))

    #try:
    #    client.connect(broker_address)
    #    client.loop_start()
    #except:
    #    print("mqtt connect error")

    try:
        print("STARTING...")
        time.sleep(1)

        p1.start()
        p2.start()
        p3.start()
        #p4.start()

        p1.join()
        p2.join()
        p3.join()
        #p4.join()

    except KeyboardInterrupt:
        # Save the Excel workbook & Close the workbook when done
        wb.close()
        print("keyboard interrupt")

    finally:
        p1.kill()
        p2.kill()
        p3.kill()
        #p4.kill()
        GPIO.cleanup()
        print("gpio cleaned up!")
        print("excel data saved!")
        print("TERMINATED...")