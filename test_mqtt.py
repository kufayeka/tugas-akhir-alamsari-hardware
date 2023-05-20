import paho.mqtt.client as mqtt

# MQTT broker details
host = '203.189.120.137'
port = 1883
username = 'petra_mqtt_broker'
password = '1n!_mqttBroker'

# Callback function for MQTT client connection
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('Connected to MQTT broker')
        # Subscribe to a topic upon successful connection
        client.subscribe('your/topic')  # Replace with your desired topic
    else:
        print('Connection failed')

# Callback function for MQTT client disconnection
def on_disconnect(client, userdata, rc):
    print('Disconnected from MQTT broker')
    

# Callback function for MQTT message reception
def on_message(client, userdata, msg):
    print('Received message:')
    print('Topic: ' + msg.topic)
    print('Message: ' + msg.payload.decode())

# Create MQTT client and set the callback functions
client = mqtt.Client("baddestbitch")
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

# Set username and password
client.username_pw_set(username, password)

# Connect to the MQTT broker
client.connect(host, port)

# Start the MQTT network loop
client.loop_start()

# Publish a sample message
client.publish('your/topic', 'Hello, MQTT!')  # Replace with your desired topic and message

# Continue other tasks or wait for messages
