#!/usr/bin/env python3
import datetime
import global_variables
import multiprocessing
import openpyxl
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import schedule
import time


from climate_sensors import read_climate_sensors
from simple_pid import PID

# multiprocessing variables manager
manager = multiprocessing.Manager()

def map_range(value, inMin, inMax, outMin, outMax):
    result = outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
    return result

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
        print("PWM Timing / DutyCycle (secs): ")
        print(f"\tHigh Time:{gv.PWM_high_time.get():.3f} | Low Time:{gv.PWM_low_time.get():.3f}")

    while True:
        sensors_job()
        time.sleep(10)

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

def process3(gv, relayPin):
    while True:
        if gv.PWM_enabled.get() == 1:
            # set the relayPin to HIGH as long as the PWM_high_time.value
            GPIO.output(relayPin, GPIO.LOW)
            time.sleep(gv.PWM_high_time.get())
            # if PWM_low_time.value != 0, set the relayPin to LOW as long as the PWM_high_time.value
            if gv.PWM_low_time.get() != 0:
                GPIO.output(relayPin, GPIO.LOW)
                time.sleep(gv.PWM_low_time.get())
        # if PWM_enabled.value == 0, set relayPin to LOW
        else:
            GPIO.output(relayPin, GPIO.LOW)
            time.sleep(1)

def process4():
    class MQTTModuleClass:
        mqtt_is_connected = False

        def __init__(self, broker_address, broker_username, broker_password):
            self.client = mqtt.Client("efeifbeifwmdd", clean_session=False)
            self.client.username_pw_set(broker_username, broker_password)
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message
            self.client.on_connect_fail = self.on_connect_fail
            try:
                self.client.connect(broker_address)
            except:
                print("connect error")
            self.client.loop_start()
            self.client.reconnect_delay_set(min_delay=1, max_delay=120)

        def on_connect(self, client, userdata, flags, rc):
            if rc == 0:
                self.mqtt_is_connected = True
                print("Connected to MQTT broker with result code " + str(rc))
                self.client.subscribe("pussy_wet")
            else:
                print("Failed to connect to Broker, return code =", rc)

        def on_disconnect(self, client, userdata, rc):
            self.mqtt_is_connected = False
            if rc != 0:
                print("Unexpected disconnection from MQTT broker")

        def on_connect_fail(self, client, userdata, rc):
            self.mqtt_is_connected = False
            print("Connection Failed")
            self.reconnect()

        def on_message(self, client, userdata, msg):
            print("Received message: " + msg.topic + " " + str(msg.payload))

        def publish_message(self, topic, message):
            self.client.publish(topic, message)

        def subscribe_topic(self, topic):
            self.client.subscribe(topic)

        def check_connection(self):
            return self.client.is_connected()

        def reconnect(self):
            self.client.connect(broker_address)
            print("reconnect")
            time.sleep(5)

        def stop(self):
            self.client.loop_stop()

    try:
        broker_address = "192.168.43.38"
        broker_username = "petra_mqtt_broker"
        broker_password = "petraMqttBroker777"

        mqtt_instance = MQTTModuleClass(broker_address, broker_username, broker_password)

        while True:
            print("Connection status:", mqtt_instance.check_connection())

            if not mqtt_instance.check_connection():
                print("reconnecting...") # reconnect already handled in the bg for loop.start()

            if mqtt_instance.mqtt_is_connected:
                mqtt_instance.publish_message("pussy_wet", "and_moist")
                time.sleep(1)
            else:
                time.sleep(1)

    except EOFError:
        print("Error")

#############################################################################
if __name__ == '__main__':
    gv = global_variables

    # setup GPIO
    relayPin = 12
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(relayPin, GPIO.OUT)

    p1 = multiprocessing.Process(target=process1, args=(gv,))
    p2 = multiprocessing.Process(target=process2, args=(gv,))
    p3 = multiprocessing.Process(target=process3, args=(gv, relayPin))
    p4 = multiprocessing.Process(target=process4, args=())

    try:
        print("STARTING...")
        p1.start()
        p2.start()
        p3.start()
        p4.start()

        p1.join()
        p2.join()
        p3.join()
        p4.join()

    except KeyboardInterrupt:
        print("keyboard interrupt")

    finally:
        p1.kill()
        p2.kill()
        p3.kill()
        p4.kill()
        GPIO.cleanup()
        print("gpio cleaned up")
        print("TERMINATED...")