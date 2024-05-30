import paho.mqtt.client as mqtt
import time

# Set up connection parameters for the source MQTT broker (localhost)
source_broker_host = "localhost"
source_broker_port = 1883
source_topic = "#"
excluded_topics = ["/led_d", "/f_control_d", "/b_control_d", "/cfco2", "/a_or_m"]

# Set up connection parameters for the destination MQTT broker
destination_broker_host = ""
destination_broker_port = 1883
destination_topic = "NamCLoud/feeds"
destination_username = "NamCLoud"
destination_password = ""

# Callback when connection is successful to the source broker
def on_connect_source(client, userdata, rc):
    print("Connected to source broker with result code {0}".format(rc))
    client.subscribe(source_topic)

# Callback when a message is received from the source broker
def on_message_source(client, userdata, msg):
    try:
        decoded_payload = msg.payload.decode('utf-8')
        cleaned_payload = decoded_payload.replace('\x00', '')
        received_topic = msg.topic
        # Check if the received topic is in the list of excluded topics
        if received_topic not in excluded_topics:
            encoded_topic = msg.topic.encode('utf-8')
            encoded_payload = decoded_payload.encode('utf-8')
            print("Received message on source topic {0}: {1}".format(encoded_topic, encoded_payload))

            # Publish the received message to the destination broker with the received topic
            client_destination.publish(destination_topic + received_topic, cleaned_payload, qos=0)
            print("Published message to destination topic {0}".format(destination_topic + received_topic))

    except UnicodeDecodeError as e:
        print("Error decoding message on topic {0}: {1}".format(msg.topic, e))
    except UnicodeEncodeError as e:
        print("Error encoding message on topic {0}: {1}".format(msg.topic, e))

# Create an MQTT client for the source broker
client_source = mqtt.Client()
client_source.on_connect = on_connect_source
client_source.on_message = on_message_source

# Connect to the source MQTT broker (localhost)
try:
    client_source.connect(source_broker_host, source_broker_port, 60)
except Exception as e:
    print("Error connecting to the source broker: {0}".format(e))
    exit()

# Callback when connection is successful to the destination broker
def on_connect_destination(client, userdata, rc):
    if rc == 0:
        print("Connected to destination broker with result code {0}".format(rc))
    else:
        print("Connection to destination broker failed with result code {0}".format(rc))

# Create an MQTT client for the destination broker
client_destination = mqtt.Client()
client_destination.on_connect = on_connect_destination
client_destination.username_pw_set(username=destination_username, password=destination_password)

# Connect to the destination MQTT broker
try:
    client_destination.connect(destination_broker_host, destination_broker_port, 60)
except Exception as e:
    print("Error connecting to the destination broker: {0}".format(e))
    exit()

# Enter an infinite loop to maintain the connection and process messages from the source broker
try:
    client_source.loop_start()
    client_destination.loop_start()
    while True:
        time.sleep(20)  # Can perform other tasks within this loop
except KeyboardInterrupt:
    print("Disconnecting from brokers...")
    client_source.disconnect()
    client_source.loop_stop()
    client_destination.disconnect()
    client_destination.loop_stop()
    print("Disconnected.")
