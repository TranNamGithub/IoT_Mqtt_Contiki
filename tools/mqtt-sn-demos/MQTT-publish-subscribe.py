#! c:\python34\python3
#!/usr/bin/env python
##demo code provided by Steve Cope at www.steves-internet-guide.com
##email steve@steves-internet-guide.com
##Free to use for any purpose
##If you like and use this code you can
##buy me a drink here https://www.paypal.me/StepenCope
import signal
import time
import paho.mqtt.client as paho
broker="broker.hivemq.com"
broker="iot.eclipse.org"
broker="192.168.1.159"
#define callback
def on_message(client, userdata, message):
   time.sleep(1)
   print("received message =",str(message.payload.decode("utf-8")))


client= paho.Client("client-001")  #create client object client1.on_publish = on_publish                          #assign function to callback client1.connect(broker,port)                                 #establish connection client1.publish("house/bulb1","on")  
######
client.on_message=on_message
#####
print("connecting to broker ",broker)
client.connect(broker)#connect
client.loop_start() #start loop to process received messages
print("subscribing ")
client.subscribe("house/+/furnace/+/temperature")#subscribe
time.sleep(2)
print("publishing ")
client.publish("house/room1/furnace/alarm1/temperature","on",True)#publish
m="test"
client.publish("house/room2/furnace/alarm2/temperature",m,True)#publish
time.sleep(4)
#client.unsubscribe("house/bulb1")
time.sleep(4)
client.disconnect() #disconnect
client.loop_stop() #stop loop
