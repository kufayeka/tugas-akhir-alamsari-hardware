import paho.mqtt.client as mqtt

class MQTTPublisher:
    def __init__(self, broker_address, client_id):
        self.client = mqtt.Client(client_id)
        self.client.username_pw_set(broker_username, broker_password)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.connect(broker_address)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection from MQTT broker")

    def on_message(self, client, userdata, msg):
        print("Received message: " + msg.topic + " " + str(msg.payload))

    def publish_message(self, topic, message):
        self.client.publish(topic, message)

    def subscribe_topic(self, topic):
        self.client.subscribe(topic)

    def stop(self):
        self.client.loop_stop()

def mqtt_publish_process(broker_address, client_id, topic, message):
    publisher = MQTTPublisher(broker_address, client_id)
    publisher.publish_message(topic, message)
    publisher.stop()

if __name__ == '__main__':
    broker_address = "mqtt.example.com"
    client_id = "my_client"
    topic = "my/topic"
    message = "Hello, MQTT!"

    processes = []
    for i in range(4):
        process = multiprocessing.Process(
            target=mqtt_publish_process,
            args=(broker_address, client_id, topic, message)
        )
        processes.append(process)
        process.start()

    for process in processes:
        process.join()
