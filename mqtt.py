import paho.mqtt.client as mqtt
import time
import json

# Informasi broker MQTT
broker_address = "203.189.122.131"
broker_username = "petra_mqtt_broker"
broker_password = "petraMqttBroker777"
topic = "petra/alamsari/kumbung_jamur/setting/all_parameters"
qos = 1
clean_session = False
clientID = "hffsfusjdfksnfjsb"

# Fungsi untuk melakukan koneksi ke broker MQTT
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code: " + str(rc))
    # Melakukan subscribe ke topik setelah terhubung
    client.subscribe(topic, qos=qos)

# Fungsi untuk menangani event publish
def on_publish(client, userdata, mid):
    print("Message published")

# Fungsi untuk menangani event menerima pesan
def on_message(client, userdata, msg):
    print("Received message: " + str(msg.payload.decode()))

# Membuat objek MQTT client
client = mqtt.Client(client_id=clientID, clean_session=clean_session)

# Mengatur username dan password broker (jika diperlukan)
client.username_pw_set(broker_username, broker_password)

# Mengatur callback function untuk event connect
client.on_connect = on_connect

# Mengatur callback function untuk event publish
client.on_publish = on_publish

# Mengatur callback function untuk event menerima pesan
client.on_message = on_message

# Melakukan koneksi ke broker MQTT
client.connect(broker_address)

payload = {
	'pid_settings': { 
        'set_point': 70, 
        'kp': 10, 
        'ki': 0.1, 
        'kd': 1, 
        'interval_from': 0, 
        'interval_to': 5 
    },
	'data_logger_settings': { 
        'interval': 60
    }
}

client.publish(topic, json.dumps(payload), qos=qos, retain=True)
time.sleep(1)

# Memproses pesan yang diterima
client.loop()
time.sleep(3)
client.loop_stop()
