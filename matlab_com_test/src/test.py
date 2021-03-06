#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def generate_Command(action,parameters,vel):
    command = action+"_param:"
    temp = ""
    for i in parameters:
        temp = temp+"_"+str(i)
    temp = command+temp+"_vel:_"+str(vel)


    print(temp)
    return

rospy.init_node('help')

pub = rospy.Publisher('matlab_kuka_test', String, queue_size=1)

test = [-400,200,600,"pi",0,"pi"]
generate_Command("Cartesian",test, 1)
