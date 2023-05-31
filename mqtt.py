import json
import time
import paho.mqtt.client as mqtt

broker_address = "203.189.122.131"
broker_username = "petra_mqtt_broker"
broker_password = "petraMqttBroker777"
topic = "petra/alamsari/kumbung_jamur/data_logger_klimat"
qos = 1
clean_session = False

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(topic)

def on_message(client, userdata, msg):
    print("Received message: " + msg.payload.decode())

def on_publish(client, userdata, mid):
    print("Message published")

client = mqtt.Client("xchjkuiyuftckg", clean_session=clean_session)
client.username_pw_set(broker_username, broker_password)
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish

client.connect(broker_address)

client.loop_start()

while True:
    payload = {
                'temp1': 25,
                'temp2': 26,
                'hum1': 80,
                'hum2': 76
            }
    client.publish(topic, json.dumps(payload), 1, False)
    time.sleep(1)

client.loop_stop()
client.disconnect()
