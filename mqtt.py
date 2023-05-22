import time
import paho.mqtt.client as mqtt

broker_address = "192.168.43.38"
broker_username = "petra_mqtt_broker"
broker_password = "petraMqttBroker777"
topic = "mytopic"
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
    message = "bad bitch"
    client.publish(topic, message, qos=qos)
    time.sleep(1)

client.loop_stop()
client.disconnect()
