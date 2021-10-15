#!/usr/bin/env python
import time
import json
import math
import rospy
import socket
import threading
import actionlib
import PyKDL
import tf

import socket
import fcntl
import struct
import uuid

# from black_maine_app.msg import DistributionAction
import geometry_msgs.msg
import diagnostic_msgs.msg

import paho.mqtt.client as mqtt

# from task_manager.srv import TaskNew
#from task_manager.srv import *
#from task_manager.msg import *

class MqttListener:

  def __init__(self):

    self.robot_id = rospy.get_param('mqtt_listener/robot_id', "truck_01")
    self.client = mqtt.Client(client_id=self.robot_id)
    self.client.on_connect = self.on_connect
    self.client.on_message = self.on_message
    self.client.connect("127.0.0.1", 1883, 60)

#    self._task_client = rospy.ServiceProxy("task_new", TaskNew)

  def UDPLinstenLoop(self):
    # rospy.loginfo("start udp linster..")
    self.client.loop_start()

  # The callback for when the client receives a CONNACK response from the server.
  def on_connect(self, client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # self.client.subscribe([("iqr/default/callbutton/state", 0), ("iqr/default/agv_manager/state", 0)])
    self.client.subscribe("iqr/default/callbutton/goal")
    self.client.message_callback_add("iqr/default/callbutton/goal", self.callback)
    # message_callback_add(sub, callback)

  def callback(self, client, userdata, msg):
    # pass
    rospy.loginfo("callbutton/goal callback ")
    print(msg.payload)

    # msg_dist = "{priority": 1, "sub_task": ["nav", "wait_for_confirm", "dock", "set_io", "dock", "nav", "wait_for_confirm"],"param": ["dock", "wait_for_confirm", "on" , "0_true", "off", "dock", "wait_for_confirm"]}"
    msg_dist = eval(msg.payload)
    priority = msg_dist["priority"]
    sub_task = msg_dist["sub_task"]
    param = msg_dist["param"]



  # The callback for when a PUBLISH message is received from the server.
  def on_message(self, client, userdata, msg):
      rospy.loginfo("mqtt receive msg ")
      # print(msg.topic+" "+str(msg.payload))
      pass


  def NewTask(self):

    task_new_msg = task_manager.srv.TaskNewRequest()
    print(type(task_new_msg.task_info))
    # task_info_msg = TaskInfo()
    task_new_msg.task_info.type = "maniplator"
    task_new_msg.task_info.priority = 1
    task_new_msg.task_info.sub_task = ["nav", "wait_for_confirm", "dock", "set_io", "dock", "nav", "wait_for_confirm"]
    task_new_msg.task_info.param = ["dock", "wait_for_confirm", "on" , "0_true", "off", "dock", "wait_for_confirm"]
    self._task_client.call(task_new_msg)


def main():
  rospy.init_node('brain_control_interface')
  ML = MqttListener()
#  ML.NewTask()
  t0 = threading.Thread(target=ML.UDPLinstenLoop,args=())
  t0.start()
  # t1 = threading.Thread(target=ML.RobotStatePublisher,args=())
  # t1.start()
  # ML.get_mac_address()
  # ML.get_ip_address('wlan0')
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
