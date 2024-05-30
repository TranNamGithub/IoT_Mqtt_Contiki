#! c:\python34\python3
#!/usr/bin/env python
##demo code provided by Steve Cope at www.steves-internet-guide.com
##email steve@steves-internet-guide.com
##Free to use for any purpose
##If you like and use this code you can
##buy me a drink here https://www.paypal.me/StepenCope
import signal
import queue
import threading
import time
import sys
sys.path.append('c:/python34/steve/mqttsclient/client')
print(sys.version_info)
from MQTTSNclient import Callback
from MQTTSNclient import Client
#from MQTTSNclient import publish
import MQTTSN
message_q=queue.Queue()

class MyCallback(Callback):
  def on_message(self,client,TopicId,Topicname, payload, qos, retained, msgid):
    m= "Arrived" +" topic  " +str(TopicId)+ "message " +\
       str(payload) +"  qos= " +str(qos) +" ret= " +str(retained)\
       +"  msgid= " + str(msgid)
    #print("got the message ",payload)
    message_q.put(payload)
    return True



######
def empty_queue(delay=0):
    while not message_q.empty():
      m=message_q.get()
      print("Received message  ",m)
    if delay!=0:
      time.sleep(delay)
########


host="192.168.1.41"
port=10000
client = Client("linh2")#change so only needs name
client.message_arrived_flag=False
client.registerCallback(MyCallback())
print ("threads ",threading.activeCount()) 
print("connecting ",host)
client.connected_flag=False

client.connect(host,port)

client.lookfor(MQTTSN.CONNACK)
try:
  if client.waitfor(MQTTSN.CONNACK)==None:
      print("connection failed")
      raise SystemExit("no Connection quitting")
except:
  print("connection failed")
  raise SystemExit("no Connection quitting")
client.loop_start() #start loop

"""
#Alternative form

while not client.connected_flag:
  time.sleep(1)
  print("waiting for connection")

"""
#quit()
  

print ("threads ",threading.activeCount()) 
topic1="testtopic"
print("topic for topic1 is", topic1)
print("connected now subscribing")
client.subscribed_flag=False
while True:
  topic1_id,rc = client.subscribe(topic1,1)
  if rc==None:
    print("subscribe failed")
    raise SystemExit("Subscription failed quitting")
  if rc==0:
    print("subscribed ok to ",topic1)
    break

count=0
msg="test message"

try:
  while True:
    nmsg=msg+str(count)
    count+=1
    print("publishing message ",topic1_id)
    id= client.publish(topic1_id,nmsg,qos=0)
    time.sleep(1)
    empty_queue(0)
    pass
except KeyboardInterrupt:
    print ("You hit control-c")

print ("threads ",threading.activeCount()) 

print("disconnecting")
client.disconnect()
client.loop_stop()
