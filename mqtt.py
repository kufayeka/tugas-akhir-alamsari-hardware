import paho.mqtt.client as mqtt

class MQTTModuleClass:
    def __init__(self, broker_address, broker_username, broker_password):
        self.client = mqtt.Client("efeifbeifwmdd")
        self.client.username_pw_set(broker_username, broker_password)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_connect_fail = self.on_connect_fail
        self.client.connect(broker_address)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection from MQTT broker")
    
    def on_connect_fail(self, client, userdata, rc):
        print("Connection Failed")

    def on_message(self, client, userdata, msg):
        print("Received message: " + msg.topic + " " + str(msg.payload))

    def publish_message(self, topic, message):
        self.client.publish(topic, message)

    def subscribe_topic(self, topic):
        self.client.subscribe(topic)
    
    def fuck_me_good_daddy(self, x):
        print("love me good daddy", x)

    def stop(self):
        self.client.loop_stop()