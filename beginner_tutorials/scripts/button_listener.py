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
from  task_manager.msg import ButtonState
from  task_manager.srv import TaskNewRequest
from  task_manager.srv import TaskNew
import paho.mqtt.client as mqtt


button_dict={}

class StateListener:
  #global button_dict
  
  def __init__(self):
    self.__sub_topic_name=rospy.get_param('~sub_topic_name',"iqr/callbutton/state")
    self.__sub_topic_name2=rospy.get_param('~sub_topic_name2',"iqr/callbutton/goal")
    self.__time_out=rospy.get_param('~time_out',10)
    self.__looprate=rospy.get_param('~looprate',1)
    self.robot_id = rospy.get_param('~mqtt_listener/client_id', "truck_01")
    self.client = mqtt.Client(client_id=self.robot_id)
    self.client.on_connect = self.on_connect
    self.client.on_message = self.on_message
    self.client.connect("192.168.40.122", 1883, 60)
    self.pub = rospy.Publisher('callbutton/state', ButtonState, queue_size=10)
    self._task_client = rospy.ServiceProxy("task_new", TaskNew)
    


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
    self.client.subscribe([(self.__sub_topic_name,0),(self.__sub_topic_name2,0)])
    self.client.message_callback_add(self.__sub_topic_name, self.callback)
    self.client.message_callback_add(self.__sub_topic_name2, self.callback2)
    # message_callback_add(sub, callback)

  def callback(self, client, userdata, msg):
    
    # pass
    rospy.loginfo("callbutton/state callback ")
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
  

  def callback2(self, client, userdata, msg):
    
    # pass
    rospy.loginfo("callbutton/state callback ")
    #print(msg.payload)
    null="wrong"
    # msg_dict = "{"message_type": "state", "id": "btn_01", "state": "ready", "ip": "192.168.0.123", "mac_address": "FD:FD:FD:FD:FD:FD"}"
    try:
      eval(msg.payload)
    except Exception as e:
      print(repr(e))
      print("there is a wrong in jeson_msg")
    else:
      task_new_msg = TaskNewRequest()
      # print(type(task_new_msg.task_info))
      msg_dict2 = eval(msg.payload)
      priority = eval(str(msg_dict2["priority"]))
      sub_task = eval(str(msg_dict2["sub_task"]))
      param = eval(str(msg_dict2["param"]))
      print(msg_dict2)
      print(priority)
      print(sub_task)
      print(param)
      task_new_msg.type = "maniplator"
      task_new_msg.priority = priority
      task_new_msg.sub_task = sub_task
      task_new_msg.param = param
      self._task_client.call(task_new_msg)



  # The callback for when a PUBLISH message is received from the server.
  def on_message(self, client, userdata, msg):
      rospy.loginfo("mqtt receive msg ")
      # print(msg.topic+" "+str(msg.payload))
      pass

  def StatePubLoop(self):
    lock = threading.Lock()
    count = 0
    ButtonState_id_list=[]
    ButtonState_ip_list=[]
    ButtonState_state_list=[]
    ButtonState_mac_list=[]
    #looprate
    ButtonState_msg=ButtonState()
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
          ButtonState_id_list.append(single_button_dict["id"])
          ButtonState_ip_list.append(single_button_dict["ip"])
          ButtonState_state_list.append(single_button_dict["state"])
          ButtonState_mac_list.append(single_button_dict["mac_address"])

          ButtonState_msg.id=ButtonState_id_list
          ButtonState_msg.ip=ButtonState_ip_list
          ButtonState_msg.state=ButtonState_state_list
          ButtonState_msg.mac_address=ButtonState_mac_list
        ButtonState_msg.header.seq = count
        ButtonState_msg.header.stamp =rospy.Time.now()
        ButtonState_msg.header.frame_id = "ButtonState_msg"
        self.pub.publish(ButtonState_msg)
        ButtonState_id_list=[]
        ButtonState_ip_list=[]
        ButtonState_state_list=[]
        ButtonState_mac_list=[]
        count=count+1
      rate.sleep()    
        #print(key_list)
        #print(button_dict)
         
    
def main():
  rospy.init_node('button_listener')
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