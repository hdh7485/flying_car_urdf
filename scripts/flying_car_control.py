#!/usr/bin/env python

import math
import numpy
import threading

from math import pi

import rospy
import tf

from std_msgs.msg import Float64
from std_msgs.msg import String
from controller_manager_msgs.srv import ListControllers

current_mode = "FLIGHT"
FL_arm_pub = rospy.Publisher('/FL_arm_position_controller/command', Float64, queue_size=1)
FR_arm_pub = rospy.Publisher('/FR_arm_position_controller/command', Float64, queue_size=1)
RL_arm_pub = rospy.Publisher('/RL_arm_position_controller/command', Float64, queue_size=1)
RR_arm_pub = rospy.Publisher('/RR_arm_position_controller/command', Float64, queue_size=1)

FL_prop_pub = rospy.Publisher('/FL_prop_position_controller/command', Float64, queue_size=1)
FR_prop_pub = rospy.Publisher('/FR_prop_position_controller/command', Float64, queue_size=1)
RL_prop_pub = rospy.Publisher('/RL_prop_position_controller/command', Float64, queue_size=1)
RR_prop_pub = rospy.Publisher('/RR_prop_position_controller/command', Float64, queue_size=1)

def unfold():
    unfold_value = Float64(0)
    FL_arm_pub.publish(unfold_value)
    FR_arm_pub.publish(unfold_value)
    RL_arm_pub.publish(unfold_value)
    RR_arm_pub.publish(unfold_value)

def fold():
    FL_prop_pub.publish(-0.4)
    FR_prop_pub.publish(-0.8)
    RL_prop_pub.publish(-0.4)
    RR_prop_pub.publish(-0.8)
    fold_value = Float64(-1.56)
    FL_arm_pub.publish(-1.56)
    FR_arm_pub.publish(-1.56)
    RL_arm_pub.publish(-1.46)
    RR_arm_pub.publish(-1.46)

def callback(data):
    if data.data == "FLIGHT":
        unfold()
    elif data.data == "DRIVE":
        fold()


rospy.init_node('flying_car_control', anonymous=True)
rospy.Subscriber("mode_command", String, callback)
rospy.spin()
