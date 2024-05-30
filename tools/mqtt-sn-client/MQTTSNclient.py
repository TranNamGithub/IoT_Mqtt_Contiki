"""
/*******************************************************************************
 * Copyright (c) 2011, 2013 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution. 
 *
 * The Eclipse Public License is available at 
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at 
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/
"""
#add by me
import queue
import struct
import threading
import random
import time

##

import MQTTSN,socket, time, MQTTSNinternal, _thread, sys,types
#, struct
debug=False
logging=False
MQTTSNinternal.debug=False
      
class Callback:

  def __init__(self):
    self.events = []
    #self.registered = {} #added to client object
  def on_subscribe(self,client,TopicId,MsgId,rc):
    if debug:
      print("suback",TopicId,MsgId,rc)
    client.suback_flag=True
    for t in client.topics:
      if t[1]==MsgId:
          t[2]=1 #acknowledged
          m="subscription acknowledged  "+str(t[0])
          if debug:
            print(m)
    client.sub_topicid=TopicId
    client.sub_msgid=MsgId
    client.sub_rc=rc

  def on_connect(self,client,address,rc):
    if debug:
      print("in on connect")
    if rc==0:
      client.connected_flag=True
    else:
      client.bad_connect_flag=True
  def on_disconnect(self,client,cause):

    if debug:
      print("default connection Lost", cause)
    self.events.append("disconnected")
    client.connected_flag=False

  def on_message(self,client,TopicId,Topicname, payload, qos, retained, msgid):
    if TopicId !=0:
      m= "on-message-Arrived" +" topic  " +str(TopicId)+ "message " +\
         str(payload) +"  qos= " +str(qos) +" ret= " +str(retained)\
         +"  msgid= " + str(msgid)
    else:
      m= "on-message-Arrived" +" topic  " +str(Topicname)+ "message " +\
       str(payload) +"  qos= " +str(qos) +" ret= " +str(retained)\
       +"  msgid= " + str(msgid)

    if debug or client.logging:
      print(m)
    m={"msg":payload,"msgid":msgid,"topicid":TopicId,"qos":qos}
    client.messages.append(m)


    
  def on_puback(self,client,msgId,rc,qos):
    print("in pub ack msgId=",msgId, "rc= ",rc)
    client.puback_flag=True

  def deliveryComplete(self, msgid):
    print("default deliveryComplete")
    pass
  
  def on_advertise(self,client,address, gwid, duration):
    m="advertise -address" + str(address)+"qwid= " +str(gwid)+"dur= " +str(duration)
    if debug:
      print ("found gateway at from advertise message",m)
    ip_address,port=address
    client.add_gateway(str(gwid),ip_address,port,str(duration))
    client.GatewayFound=True

  def gwinfo(self,client,address,gwid,gwadd):
    ip_address,port=address
    if gwadd:
        print("In gateway info " +gwadd)
        ip_address=gwadd #need to swap as this message came from a client

    client.add_gateway(gwid,ip_address,port,0)

  def on_register(self,client, TopicId, TopicName):
    if debug:
      print("Topic name=",TopicName, "id =",TopicId)

    client.registered[TopicId] = TopicName
    
  def on_regack(self,client, TopicId):
    print("in regack id =",TopicId)
    client.registered_topicid=TopicId
    client.registered_topic_flag=True
  def on_willtopicresp(self,client,rc):
      print("will topic returned ",rc)

  def on_willmsgresp(self,client,rc):
      print("will message returned ",rc)




class Client:
    def __init__(self, clientid="",cleansession=True):
      #self.message_q=queue.Queue()
      self.logging=False
      self.multicast_flag=False
      self.registered = {}
      self.running_loop=False #set when starting external loop
      self.clientid = clientid
      if clientid=="":
        a=random.randint(0,1000)
        clientid="testclient-"+str(a)
      self.gateway=("",1883)
      self.gateways=[]
      self.GatewayFound=False
      self.topics={}
      self.inMsgTime = 0
      self.outMsgTime = 0
      self.cleansession=cleansession
      self.msgid = 1
      self.messages=[] #used to store incoming messages
      self.queue=None #if set store messages in a queue
      self.callback = None
      self.connected_flag=False #added
      self.bad_connection_flag=False
      self.puback_flag=False #added
      self.suback_flag=False
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
      self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      self.sock.settimeout(0.001)
      self.__receiver = MQTTSNinternal.Receivers(self.sock,self)

      self.sub_topicid=""
      self.sub_msgid=""
      self.sub_rc=""
      self.send_gwsearch_timer=0
      self.send_gwinfo_timer=0
      self.duration=60
      self.ping_count=0
      self.registered_topicid=0
      self.registered_topic_flag=False
      self.will_topic=""
      self.will_msg=""
      self.multicast_group=""
      self.multicast_port=""
      #self.registerCallback(Callback())
    ##added by me create gwifo for sending
    def searchgw(self,client,address,packet):
      'need to send gw info packet'
      now=time.time()
      if now<(self.send_gwinfo_timer+5):
        return
      gwinfo = MQTTSN.GWInfos()
      if debug:
        print("sending_gwinfo ")
      for i in range(len(self.gateways)):
        if self.gateways[i]["active"]:
          host=(self.gateways[i]["host"],self.gateways[i]["port"])
          print("host ",str(host))
          gwinfo.GwAdd=self.gateways[i]["host"]
          gwinfo.GwId=self.gateways[i]["gwid"]
          gwinfo.GwId=2
          m=gwinfo.pack()
          m= m.encode() #turn to bytes
          self.sock_multi.sendto(m,(self.multicast_group,self.multicast_port))
          self.send_gwinfo_timer=time.time()
          break
    def set_gateway(self,gwid,status=False):
      'set gateway to active or inactive'
      for i in range(len(self.gateways)):
          if gwid ==self.gateways[i]["gwid"]:
            self.gateways[i]["status"]=status
    
    def add_gateway(self,gwid,host,port,duration):
      'keep track of gateways'
      add_flag=True
      for i in range(len(self.gateways)):
          if gwid ==self.gateways[i]["gwid"]:
              add_flag=False
              self.set_gateway(gwid,True)#mark active
              if debug or logging:
                print("already found this one")
                break  
      if add_flag:
          self.gateways.append({"gwid":gwid,"host":host,\
                                      "port":port,"duration":duration,"status":True})
      self.GatewayFound=True
    """   
    def send_gwinfo(self,multicast_group,multicast_port):
        now=time.time()

        if now<(self.send_gwinfo_timer+5):
          print("return")
          return
        gwinfo = MQTTSN.GWInfos()
        if debug:
          print("sending_gwinfo ")
        for i in range(len(self.gateways)):
          if self.gateways[i]["active"]:
            host=(self.gateways[i]["address"],self.gateways[i]["port"])
            print("host ",str(host))
            gwinfo.GwAdd=self.gateways[i]["address"]
            gwinfo.GwId=self.gateways[i]["gwid"]
            #gwinfo.GwId=2
            m=gwinfo.pack()
            m= m.encode() #turn to bytes
            self.sock_multi.sendto(m,(multicast_group, multicast_port))
            self.send_gwinfo_timer=time.time()
            break
    """
            

    def Search_GWs(self,multicast_address,multicast_port):
        'sends a SEARCHGW packet'
        if  not self.multicast_flag:
          self.create_multicast_socket(multicast_port,multicast_address)
         
        searchgw = MQTTSN.SearchGWs()
        m=searchgw.pack()
        m= m.encode() #turn to bytes
        self.sock_multi.sendto(m,(multicast_address, multicast_port))

    def create_multicast_socket(self,multicast_port,multicast_group):
        server_address = ('', multicast_port)
        socket.setdefaulttimeout(0.01)
        group = (multicast_group, multicast_port)
        sock_multi = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock_multi.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        mreq = struct.pack('4sL', socket.inet_aton(multicast_group), socket.INADDR_ANY) 

        sock_multi.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock_multi.bind(server_address)#receive advertise mssages
        #create new receiver for multicast

        self.__receiver_gw = MQTTSNinternal.Receivers(sock_multi,self)
        self.multicast_flag=True
        self.sock_multi=sock_multi

    """
    def find_gateway(self,multicast_group,multicast_port):
        print("in start ",multicast_group)
        self.GatewayFound=False
        if not self.multicast_flag:
          self.create_multicast_socket(multicast_port,multicast_group)
    """       


    def start(self):
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
      self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      self.sock.settimeout(0.01)

      group = socket.inet_aton(m_group)
      mreq = struct.pack('4sL', group, socket.INADDR_ANY)  
      self.sock.bind(('',m_port))
      self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
      self.startReceiver() 

      

    def __nextMsgid(self):
        def getWrappedMsgid():
            id = self.msgid + 1
            if id == 65535:
                id = 1
            if debug:
                print("in next message id -returning id=", id)
                print("returning",id)
            return id

        if len(self.__receiver.outMsgs) >= 65535:
            raise "No slots left!!"
        else:
            self.msgid = getWrappedMsgid()
            while self.msgid in self.__receiver.outMsgs:
                self.msgid = getWrappedMsgid()
        return self.msgid


    def registerCallback(self, callback):
        self.callback = callback
    def pingreq(self):
      pingreq =MQTTSN.Pingreqs()
      pingreq.ClientId=self.clientid
      self.send(pingreq.pack().encode())
      if debug:
        print("PINGREQ")
    def pingresp(self,client,packet):
      if debug:
        print("PINGRESP") 
      self.inMsgTime = time.time()
      self.ping_count=0
    def send(self,data):
      self.outMsgTime=time.time()
      self.sock.send(data)
      #sends data on socket
    def check_ping(self):
      if not self.connected_flag:
        return
      now=time.time()
      if now >(self.inMsgTime+self.duration) and now >(self.outMsgTime+self.duration):
        self.inMsgTime=now
        self.outMsgTime=now
        self.ping_count+=1
        self.pingreq()
        if self.ping_count>=2:
          self.loop_stop()
          if self.connected_flag:
            self.disconnect()
      


    def connect(self,host="localhost",port=1883,duration=60,cleansession=True,will=False):
        'accepts host,port,duration,celansession,will flag'

        self.host = host
        self.port = port
        self.bad_connect_flag=False
        self.connected_flag=False
        self.duration=duration
        self.cleansession=cleansession
        self.sock.connect((self.host, self.port))
        print("clean sessions=",self.cleansession)
        connect = MQTTSN.Connects()
        connect.ClientId = self.clientid
        connect.Flags.CleanSession = self.cleansession
        connect.duration = self.duration
        connect.Flags.Will=will
        self.send(connect.pack().encode())






  
    def lookfor(self,msgType):
        if self.__receiver:
          self.__receiver.lookfor(msgType)


    def waitfor(self, msgType, msgId=None): #old
        msg = self.__receiver.waitfor(msgType, msgId)
        #print("the message is",msg)
        return msg
    



    def subscribe(self, topic, qos=0):
        'returns message Id and topic type,normal,short,predefined'
        self.suback_flag=False
        self.sub_topicid=""
        self.sub_msgid=""
        self.sub_rc=""
        subscribe = MQTTSN.Subscribes()
        subscribe.MsgId = self.__nextMsgid()
        if type(topic) is str:
            #print("topic is string  ",topic)
            subscribe.TopicName = topic
            if len(topic) > 2:
                subscribe.Flags.TopicIdType = MQTTSN.TOPIC_NORMAL
            else:
                subscribe.Flags.TopicIdType = MQTTSN.TOPIC_SHORTNAME
        else:
            subscribe.TopicId = topic # should be int
            subscribe.Flags.TopicIdType = MQTTSN.TOPIC_PREDEFINED
        subscribe.Flags.QoS = qos
        self.send(subscribe.pack().encode())
        if self.__receiver:
          self.__receiver.lookfor(MQTTSN.SUBACK)
          msg=self.waitfor(MQTTSN.SUBACK, subscribe.MsgId)
        if msg:
          #if using shortname then return the name not topic id as it is always=0
          if subscribe.Flags.TopicIdType == MQTTSN.TOPIC_SHORTNAME:
            msg.TopicId=topic
          
          return (msg.TopicId,msg.ReturnCode)
        else:
            return None


    def unsubscribe(self, topic):
        unsubscribe = MQTTSN.Unsubscribes()
        unsubscribe.MsgId = self.__nextMsgid()
        if type(topic) is str:
            print("unsubscribe topic is string  ",topic," is ",unsubscribe.MsgId)
            unsubscribe.TopicName = topic
            if len(topic) > 2:
                unsubscribe.Flags.TopicIdType = MQTTSN.TOPIC_NORMAL
            else:
                umsubscribe.Flags.TopicIdType = MQTTSN.TOPIC_SHORTNAME
        else:
            unsubscribe.TopicId = topic # should be int
            unsubscribe.Flags.TopicIdType = MQTTSN.TOPIC_PREDEFINED
        if self.__receiver:
          self.__receiver.lookfor(MQTTSN.UNSUBACK)

          self.send(unsubscribe.pack().encode())
          msg = self.waitfor(MQTTSN.UNSUBACK, unsubscribe.MsgId)
          return unsubscribe.MsgId
  
  
    def register(self, topicName):
        register = MQTTSN.Registers()
        register.TopicName = topicName
        if len(topicName) <=2:
          print("no need to register short topic name")
          return -1
        
        if self.__receiver:#this uses callbacks
          self.__receiver.lookfor(MQTTSN.REGACK)
          #print("\n\nsending register ",register.pack(),"\n\n")
          self.send(register.pack().encode())
          self.lookfor(MQTTSN.REGACK)
          msg = self.waitfor(MQTTSN.REGACK, register.MsgId)
          if msg:
            return (msg.TopicId,msg.ReturnCode)
          else:
            return None



    def publish(self, topic, payload, qos=0, retained=False):
        publish = MQTTSN.Publishes()
        publish.Flags.QoS = qos
        publish.Flags.Retain = retained
        self.puback_flag=False #reset flag
        if isinstance(payload, str):
          local_payload = payload.encode('utf-8')
        elif isinstance(payload, (bytes, bytearray)):
          local_payload = payload
        elif isinstance(payload, (int, float)):
          local_payload = str(payload).encode('ascii')
        elif payload is None:
          local_payload = b''
        else:
          raise TypeError(
            'payload must be a string, bytearray, int, float or None.')

        if len(local_payload) > 64000:
          raise ValueError('Payload too large.')
        if type(topic) == str:
          if len(topic) > 2:
            publish.Flags.TopicIdType = MQTTSN.TOPIC_NORMAL
            publish.TopicId = len(topic)
            payload = topic + payload
            print("Error Topic too long")
            return
          else:
            publish.Flags.TopicIdType = MQTTSN.TOPIC_SHORTNAME
            publish.TopicName = topic
            #print("shortname",publish.TopicId)
        else:
            publish.Flags.TopicIdType = MQTTSN.TOPIC_NORMAL
            publish.TopicId = topic #topic is a topic id
            #print("using topic id")
        if qos in [-1, 0]: #qos 0 or -1
            publish.MsgId = 0
        else:
            publish.MsgId = self.__nextMsgid()
            #print("MsgId=", publish.MsgId)

        publish.Data = payload
        a=publish.pack()
        self.send(a)
        self.__receiver.outMsgs[publish.MsgId] = publish
        return publish.MsgId

    def disconnect(self,duration=None):
        disconnect = MQTTSN.Disconnects()
        self.send(disconnect.pack().encode("utf-8"))
        self.connected_flag=False



    def loop_start_gw(self):#looks for incoming gateway messages
        if self.callback:#only if we have defined callbacks
          #print("starting gw loop")
          t = threading.Thread(target=self.__receiver_gw,args=(self.callback,)) #start
          t.start() #start thread
          self.multicast_flag=True
 
    def loop_stop_gw(self):#looks for incoming gateway messages
      try:
        self.multicast_flag=False
      except:
        print("ignoring error")
        pass

    
    def loop_start(self):#looks for incoming messages
        if self.running_loop: #already created so return
          return
        if self.callback:#only if we have defined callbacks
          self.running_loop=True
          t = threading.Thread(target=self.__receiver,args=(self.callback,)) #start
          t.start() #start thread
          #id = _thread.start_new_thread(self.__receiver, (self.callback,))
          
    def loop_stop(self):
        self.running_loop=False 
        assert self.__receiver.inMsgs == {}
        #assert self.__receiver.outMsgs == {}
        #self.__receiver = None
        print("stopping")
        self.loop_stop_gw()
        print("flags ",self.running_loop,"  ", self.multicast_flag)
    def loop(self,interval=.001):
        self.interval=interval #not used
        if self.running_loop: #already created external so return
          return
        self.__receiver.receive(self.callback)
        
    def set_will(self,will_msg,will_topic,will_qos=0,will_retained=False):
      'paramters: will message,topic,qos,retained flag'
      self.will_topic=will_topic
      self.will_msg=will_msg
      self.will_qos=will_qos
      self.will_retained=will_retained
    def update_will_topic(self,will_topic,will_qos=0,will_retained=False):
      'paramters: will topic,qos,retained flag'
      self.will_topic=will_topic
      self.will_qos=will_qos
      self.will_retained=will_retained
      willtopicupdate =MQTTSN.WillTopicUpds()
      willtopicupdate.flags.QoS=self.will_qos
      willtopicupdate.flags.Retain=self.will_retained
      self.send(willtopicupdate.pack().encode())
    def update_will_msg(self,will_msg):
      'paramters: will message'
      self.will_msg=will_msg
      willmsgupdate =MQTTSN.WillMsgUpds()
      willmsgupdate.WillMsg=self.will_msg
      self.send(willmsgupdate.pack().encode())
    ##    
    def willtopic(self,client,msg):
      'Send will topic in response to a request from server'
      willtopicresponse =MQTTSN.WillTopics()
      willtopicresponse.flags.QoS=self.will_qos
      willtopicresponse.flags.Retain=self.will_retained
      willtopicresponse.WillTopic=self.will_topic
      print("will topic requested")
      self.send(willtopicresponse.pack().encode())
    def willmsg(self,client,msg):
      'Send will message in response to a request from server'
      print("will message requested")
      willmsgresponse =MQTTSN.WillMsgs()
      willmsgresponse.WillMsg=self.will_msg
      self.send(willmsgresponse.pack().encode())
    ##


    def willtopicreq(self,client,msg):
      willtopicresponse =MQTTSN.WILLTOPICRESP()
      willmsgresponse.WillMsg=self.will_message
      pass
    def willmsgreq(self,client,msg):
      willmsgresponse =MQTTSN.WILLMSGRESP()
      



def publish(host,port,topic,payload,retained=False):
  publish = MQTTSN.Publishes()
  publish.Flags.QoS = 3
  publish.Flags.Retain = retained
  if isinstance(payload, str):
    local_payload = payload.encode('utf-8')
  elif isinstance(payload, (bytes, bytearray)):
    local_payload = payload
  elif isinstance(payload, (int, float)):
    local_payload = str(payload).encode('ascii')
  elif payload is None:
    local_payload = b''
  else:
    raise TypeError(
        'payload must be a string, bytearray, int, float or None.')

  if len(local_payload) > 64000:
    raise ValueError('Payload too large.')
  if type(topic) == str:
    if len(topic) > 2:
      print("Error Topic too long")
      return
    else:
      publish.Flags.TopicIdType = MQTTSN.TOPIC_SHORTNAME
      publish.TopicName = topic
      #print("shortname",publish.TopicId)
  else:
    publish.Flags.TopicIdType = MQTTSN.TOPIC_NORMAL
    publish.TopicId = topic #topic is a topic id
    #print("publish using topic id")

  publish.MsgId = 0
  publish.Data = payload
  a=publish.pack()
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.sendto(a.encode(), (host, port))
  sock.close()
  return 
########




