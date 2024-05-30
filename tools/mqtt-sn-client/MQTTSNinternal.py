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

import MQTTSN, time, sys, socket, traceback



class Receivers:


  def __init__(self, socket,client):
    #print("creating receiver object")
    self.socket = socket
    self.client=client
    self.connected = False
    self.running_loop = False
    self.observe = None
    self.observed = []
    self.outMsgs={}
    self.inMsgs={}
    self.puback = MQTTSN.Pubacks()
    self.pubrec = MQTTSN.Pubrecs()
    self.pubrel = MQTTSN.Pubrels()
    self.pubcomp = MQTTSN.Pubcomps()
    self.debug =False
    self.logging = False

  def lookfor(self, msgType):
    self.observe = msgType
    if self.debug:
      pass
      #print("looking for ",str(msgType),"  ",MQTTSN.packetNames[msgType])

  def waitfor(self, msgType, msgId=None):
    m = "waitfor -waiting for " + str(msgType)+ MQTTSN.packetNames[msgType]+"\n"
    if self.debug:
      print(m)
    msg = None
    count = 0
    while True:
      #print("waiting ",str(msgType)," count", count," ",self.running_loop)
      if not self.client.running_loop: #external loop not started manual check
        #print("manual loop ",count)
        self.receive()
      while len(self.observed) > 0:   
        msg = self.observed.pop(0)
        if msg.mh.MsgType == msgType and (msgId == None or msg.MsgId == msgId):

          break
        else:
          msg = None
      if msg != None:
        break
      time.sleep(0.2)
      count += 1
      if count >= 25:
        msg = None
        break
    self.observe = None
    return msg

  def receive(self, callback=None):

    packet = None
    try:
      packet, address = MQTTSN.unpackPacket(MQTTSN.getPacket(self.socket))
      #print("\nReceived packet data =",packet,"\n")
    except Exception as e:
      if sys.exc_info()[0] != socket.timeout:
        print("getting packet unexpected exception", sys.exc_info())
        raise Exception("Packet receive error ",e)

    if packet == None:
      return
    elif self.debug or self.logging:
      print("\nReceived packet data =",packet,"\n")
      #print("address =",address)

    self.client.inMsgTime=time.time()
    if self.observe == packet.mh.MsgType:
      #print("found what we were looking for",self.observe)
      self.observed.append(packet)
      return
    if packet.mh.MsgType == MQTTSN.CONNACK:
      if hasattr(callback,"on_connect"):
        callback.on_connect(self.client,address,packet.ReturnCode)
    elif packet.mh.MsgType == MQTTSN.DISCONNECT:
      if hasattr(callback, "on_disconnect"):
        callback.on_disconnect(self.client,packet.Duration)
    elif packet.mh.MsgType == MQTTSN.SUBACK:##added by me
      if hasattr(callback, "on_subscribe"):
        callback.on_subscribe(self.client,packet.TopicId,packet.MsgId,packet.ReturnCode)
    elif packet.mh.MsgType == MQTTSN.UNSUBACK:##added by me
      if hasattr(callback, "on_unsubscribe"):
        callback.on_unsubscribe(self.client,packet.MsgId)    


    elif packet.mh.MsgType == MQTTSN.REGISTER:
      if callback and hasattr(callback, "on_register"):
        callback.on_register(self.client,packet.TopicId, packet.TopicName)
    elif packet.mh.MsgType == MQTTSN.REGACK:
      if callback and hasattr(callback, "on_regack"):
        #print("received regack")
        callback.on_regack(self.client,packet.TopicId,packet.MsgId,\
                           packet.ReturnCode)
    elif packet.mh.MsgType == MQTTSN.PUBACK:
      "check if we are expecting a puback"
      if packet.MsgId in self.outMsgs and \
        self.outMsgs[packet.MsgId].Flags.QoS == 1:
        del self.outMsgs[packet.MsgId]
        if hasattr(callback, "on_puback"):
          callback.on_puback(self.client,packet.MsgId,packet.ReturnCode,1)#qos=1
      else:
        raise Exception("No QoS 1 message with message id "+str(packet.MsgId)+" sent")

    elif packet.mh.MsgType == MQTTSN.PUBREC:
      if packet.MsgId in self.outMsgs:
        self.pubrel.MsgId = packet.MsgId
        self.socket.send(self.pubrel.pack())
      else:
        raise Exception("PUBREC received for unknown msg id "+ \
                    str(packet.MsgId))

    elif packet.mh.MsgType == MQTTSN.PUBREL:
      "release QOS 2 publication to self.client, & send PUBCOMP"
      msgid = packet.MsgId
      if msgid not in self.inMsgs:
        pass # what should we do here?
      else:
        pub = self.inMsgs[packet.MsgId]
        if callback == None or \
           callback.on_message(self.client,pub.TopicId, pub.Data, 2, pub.Flags.Retain, pub.MsgId):
          del self.inMsgs[packet.MsgId]
          self.pubcomp.MsgId = packet.MsgId
          self.socket.send(self.pubcomp.pack())
        if callback == None:
          return (pub.pub.TopicId, pub.Data, 2, pub.Flags.Retain, pub.MsgId)

    elif packet.mh.MsgType == MQTTSN.PUBCOMP:
      #"finished with this message id"
      if packet.MsgId in self.outMsgs:
        del self.outMsgs[packet.MsgId]
        if hasattr(callback, "published"):
          callback.published(self.client,packet.MsgId)
      else:
        raise Exception("PUBCOMP received for unknown msg id "+ \
                    str(packet.MsgId))

    elif packet.mh.MsgType == MQTTSN.PUBLISH:
      #"finished with this message id ?"
      #print("here  ",packet)
      qos = packet.Flags.QoS
      Topicname = packet.TopicName
      TopicId=packet.TopicId
      data = packet.Data

      if packet.Flags.QoS in [0, 3]:
        if qos == 3:
          qos = -1
          #print("in pub",MQTTSN.TOPICID)
          if packet.Flags.TopicIdType == MQTTSN.TOPICID:
            topicname = packet.Data[packet.TopicId]
            data = packet.Data[packet.TopicId]
        if callback == None:
          return (TopicId, data, qos, packet.Flags.Retain, packet.MsgId)
        else:
          callback.on_message(self.client,TopicId,Topicname, data, qos,\
                              packet.Flags.Retain, packet.MsgId)
      elif packet.Flags.QoS == 1:
        if callback == None:
          return (packet.TopicId, packet.Data, 1,
                           packet.Flags.Retain, packet.MsgId)
        else:
          if callback.on_message(self.client,TopicId,Topicname,data, 1,
                           packet.Flags.Retain, packet.MsgId):
            self.puback.MsgId = packet.MsgId
            self.socket.send(self.puback.pack().encode())
      elif packet.Flags.QoS == 2:
        self.inMsgs[packet.MsgId] = packet
        self.pubrec.MsgId = packet.MsgId
        self.socket.send(self.pubrec.pack())

    elif packet.mh.MsgType == MQTTSN.SEARCHGW: ##added by me
        self.client.searchgw(self.client,address,packet)
    elif packet.mh.MsgType == MQTTSN.ADVERTISE:
      if hasattr(callback, "on_advertise"):
        callback.on_advertise(self.client,address, packet.GwId, packet.Duration)
    elif packet.mh.MsgType == MQTTSN.GWINFO: ##added by me
        if hasattr(callback, "gwinfo"):
          callback.gwinfo(self.client,address,packet.GwId,packet.GwAdd)
    elif packet.mh.MsgType == MQTTSN.PINGRESP: ##added by me
          self.client.pingresp(self.client,packet)
    elif packet.mh.MsgType == MQTTSN.WILLTOPICREQ: ##added by me
        self.client.willtopic(self.client,packet)
    elif packet.mh.MsgType == MQTTSN.WILLMSGREQ: ##added by me
        self.client.willmsg(self.client,packet)
    elif packet.mh.MsgType == MQTTSN.WILLTOPICRESP: ##added by me
       if hasattr(callback,"on_willtopicresp"):
          callback.on_willtopicresp(self.client,packet.ReturnCode)

    elif packet.mh.MsgType == MQTTSN.WILLMSGRESP: ##added by me
        if hasattr(callback,"on_willmsgresp"):
          callback.on_willmsgresp(self.client,packet.ReturnCode)
    else:
      print("unexpected packet",packet)
      #raise Exception("Unexpected packet"+str(packet))
      return ""
    return packet

  def __call__(self, callback):
    try:
      while True:
        self.receive(callback)
        self.client.check_ping() #controls ping
        #time.sleep(1)
        if not self.client.running_loop and not \
          self.client.multicast_flag: #stops thread
          self.socket.close()
          print("closed socket")
          break
    except Exception as e:
        print("A unexpected exception", e)

