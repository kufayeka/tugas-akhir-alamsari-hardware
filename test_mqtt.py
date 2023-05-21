import paho.mqtt.client as mqtt
import time

broker_address = "192.168.43.38"
#broker_address = "broker.hivemq.com"
broker_username = "petra_mqtt_broker"
broker_password = "petraMqttBroker777"

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #client.subscribe("$SYS/#")
    client.subscribe("paho/temperature")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

client = mqtt.Client(client_id="1n!Cl1n3t1D_", clean_session=True, userdata=None, transport="tcp")
client.username_pw_set(broker_username, broker_password)
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_start()

while True:
    client.publish("paho/temperature", "brrrr", 0, False)
    print("publish")
    time.sleep(1)