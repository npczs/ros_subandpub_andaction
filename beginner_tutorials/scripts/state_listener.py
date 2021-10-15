#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import json
import math
import rospy
import socket
import threading

import fcntl
import struct
import uuid

import geometry_msgs.msg
import diagnostic_msgs.msg
from  beginner_tutorials.msg import Button_state

import paho.mqtt.client as mqtt


button_dict={}

class StateListener:
  #global button_dict
  
  def __init__(self):
    self.__sub_topic_name=rospy.get_param('button_state_listener/sub_topic_name',"iqr/callbutton/state")
    self.__time_out=rospy.get_param('button_state_listener/time_out',10)
    self.__looprate=rospy.get_param('~button_state_listener/looprate',1)
    self.robot_id = rospy.get_param('~mqtt_listener/client_id', "truck_01")
    self.client = mqtt.Client(client_id=self.robot_id)
    self.client.on_connect = self.on_connect
    self.client.on_message = self.on_message
    self.client.connect("127.0.0.1", 1883, 60)
    self.pub = rospy.Publisher('callbutton/state', Button_state, queue_size=10)

#    self._task_client = rospy.ServiceProxy("task_new", TaskNew)

  def UDPLinstenLoop(self):
    # rospy.loginfo("start udp linster..")
    self.client.loop_start()

  # The callback for when the client receives a CONNACK response from the server.
  def on_connect(self, client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    if rospy.is_shutdown():
      self.client.disconnect()
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # self.client.subscribe([("iqr/default/callbutton/state", 0), ("iqr/default/agv_manager/state", 0)])
    self.client.subscribe(self.__sub_topic_name)
    self.client.message_callback_add(self.__sub_topic_name, self.callback)
    # message_callback_add(sub, callback)

  def callback(self, client, userdata, msg):
    
    # pass
    rospy.loginfo("callbutton/goal callback ")
    #print(msg.payload)
    null="wrong"
    # msg_dict = "{"message_type": "state", "id": "btn_01", "state": "ready", "ip": "192.168.0.123", "mac_address": "FD:FD:FD:FD:FD:FD"}"
    try:
      eval(msg.payload)
    except Exception as e:
      print(repr(e))
      print("there is a wrong in jeson_msg")
    else:
      msg_dict = eval(msg.payload)
      msg_dict["time"]=str(rospy.get_time())   
      msg_id = msg_dict["id"]
      #msg_state = msg_dict["state"]
      #msg_ip = msg_dict["ip"]
      #msg_mac_address = msg_dict["mac_address"]
      button_dict[msg_id]=str(msg_dict)
      print(button_dict)





  # The callback for when a PUBLISH message is received from the server.
  def on_message(self, client, userdata, msg):
      rospy.loginfo("mqtt receive msg ")
      # print(msg.topic+" "+str(msg.payload))
      pass

  def StatePubLoop(self):
    lock = threading.Lock()
    count = 0
    Button_state_id_list=[]
    Button_state_ip_list=[]
    Button_state_state_list=[]
    Button_state_mac_list=[]
    #looprate
    Button_state_msg=Button_state()
    rate = rospy.Rate(self.__looprate)
    while not rospy.is_shutdown():
      if button_dict!={}:
        #button_dict[b]
        key_list=button_dict.keys()
        for i in range(len(button_dict)):
          single_button_dict=eval(button_dict[key_list[i]])
          time_now=rospy.get_time()
          dlat=time_now-float(single_button_dict["time"])
          if dlat>self.__time_out:
            lock.acquire()
            button_dict.pop(key_list[i])
            lock.release()

        button_tuple=button_dict.items()   #transform dict to tuple
        for i in range(len(button_tuple)):
          single_button_dict=eval(button_tuple[i][1])
          Button_state_id_list.append(single_button_dict["id"])
          Button_state_ip_list.append(single_button_dict["ip"])
          Button_state_state_list.append(single_button_dict["state"])
          Button_state_mac_list.append(single_button_dict["mac_address"])

          Button_state_msg.id=Button_state_id_list
          Button_state_msg.ip=Button_state_ip_list
          Button_state_msg.state=Button_state_state_list
          Button_state_msg.mac_address=Button_state_mac_list
        Button_state_msg.header.seq = count
        Button_state_msg.header.stamp =rospy.Time.now()
        Button_state_msg.header.frame_id = "Button_state_msg"
        self.pub.publish(Button_state_msg)
        Button_state_id_list=[]
        Button_state_ip_list=[]
        Button_state_state_list=[]
        Button_state_mac_list=[]
        count=count+1
      rate.sleep()    
        #print(key_list)
        #print(button_dict)
         
    
def main():
  rospy.init_node('button_state_listener')
  ML = StateListener()
#  ML.NewTask()
  t0 = threading.Thread(target=ML.UDPLinstenLoop,args=())
  t0.start()
  t1 = threading.Thread(target=ML.StatePubLoop,args=())
  t1.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
