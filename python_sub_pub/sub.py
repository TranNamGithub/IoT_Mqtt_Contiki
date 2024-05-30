import paho.mqtt.client as mqtt
import time

# Set up connection parameters for the source MQTT broker (cloud)
cloud_broker_host = ""
cloud_broker_port = 1883
cloud_topic_main = "NamCLoud/feeds/"
cloud_topics_to_subscribe = ["led_d", "f_control_d", "b_control_d", "cfco2", "a_or_m"]  # Add your topics to this list

# Set up connection parameters for the local MQTT broker
local_broker_host = "localhost"
local_broker_port = 1883
local_topic_prefix = "/"

# Callback when connection is successful to the cloud broker
def on_connect_cloud(client, userdata, rc):
    if rc == 0:
        print("Connected to cloud broker with result code {0}".format(rc))
        for topic in cloud_topics_to_subscribe:
            full_topic = cloud_topic_main + topic
            client.subscribe(full_topic)
            print("Subscribed to cloud topic: {0}".format(full_topic))
    else:
        print("Connection to cloud broker failed with result code {0}".format(rc))

# Callback when a message is received from the cloud broker
def on_message_cloud(client, userdata, msg):
    try:
        decoded_payload = msg.payload.decode('utf-8')
        received_topic = msg.topic[len(cloud_topic_main):]  # Extract the topic after "NamCLoud/feeds/"
        print("Received message on cloud topic {0}: {1}".format(msg.topic, decoded_payload))

        # Publish the received message to the local broker with a modified topic
        local_topic = local_topic_prefix + received_topic
        client_local.publish(local_topic, decoded_payload, qos=0)
        print("Published message to local topic {0}".format(local_topic))

    except UnicodeDecodeError as e:
        print("Error decoding message on topic {0}: {1}".format(msg.topic, e))

# Create an MQTT client for the cloud broker
client_cloud = mqtt.Client()
client_cloud.on_connect = on_connect_cloud
client_cloud.on_message = on_message_cloud

# Connect to the cloud MQTT broker
try:
    client_cloud.connect(cloud_broker_host, cloud_broker_port, 60)
except Exception as e:
    print("Error connecting to the cloud broker: {0}".format(e))
    exit()

# Create an MQTT client for the local broker
client_local = mqtt.Client()

# Connect to the local MQTT broker (localhost)
try:
    client_local.connect(local_broker_host, local_broker_port, 60)
except Exception as e:
    print("Error connecting to the local broker: {0}".format(e))
    exit()

# Enter an infinite loop to maintain the connection and process messages from the cloud broker
try:
    client_cloud.loop_start()
    client_local.loop_start()
    while True:
        time.sleep(20)  # Can perform other tasks within this loop
except KeyboardInterrupt:
    print("Disconnecting from brokers...")
    client_cloud.disconnect()
    client_cloud.loop_stop()
    client_local.disconnect()
    client_local.loop_stop()
    print("Disconnected.")
