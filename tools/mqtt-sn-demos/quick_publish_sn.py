#! c:\python34\python3
#!/usr/bin/env python
##demo code provided by Steve Cope at www.steves-internet-guide.com
##email steve@steves-internet-guide.com
##Free to use for any purpose
##If you like and use this code you can
##buy me a drink here https://www.paypal.me/StepenCope
import signal
import queue
import struct
import threading
import time
import sys
sys.path.append('c:/python34/steve/mqttsclient/client')
from MQTTSNclient import Callback
from MQTTSNclient import Client
from MQTTSNclient import publish
import MQTTSN


#client = Client("linh_pub")#change so only needs name

print ("threads ",threading.activeCount()) 

for i in range(10):
    
    publish("192.168.1.41",10000,"tt","test")
time.sleep(3)


