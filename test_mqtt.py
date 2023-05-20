import paho.mqtt.client as mqtt
import time

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        client.subscribe("house/bulbs/bulb1")
    else:
        print("Connection failed")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection")

def on_message(client, userdata, message):
    print("Message received:", message.payload.decode("utf-8"))
    print("Message topic:", message.topic)
    print("Message QoS:", message.qos)
    print("Message retain flag:", message.retain)

def on_publish(client, userdata, mid):
    print("Message published")

broker_address = "192.168.43.38"
client = mqtt.Client("P1")
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
client.on_publish = on_publish

print("Creating new instance")
client.connect(broker_address, port=1883)

client.loop_start()

print("Publishing message to topic: house/bulbs/bulb1")
client.publish("house/bulbs/bulb1", "OFF")

time.sleep(4)

client.loop_stop()
client.disconnect()
