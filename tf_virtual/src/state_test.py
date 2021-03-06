#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import std_msgs.msg
from std_srvs.srv import Empty

# std_msgs/Header header
# string[] name
# float64[] position
# float64[] velocity
# float64[] effort

def MATLAB_CB(req):
    print("Got request")




rospy.init_node('joint_state_test')

pub = rospy.Publisher('joint_states', JointState, queue_size=1)
rospy.Service('matlab_test_service', Empty, MATLAB_CB)


rate = rospy.Rate(10)
while not rospy.is_shutdown():
    joint_temp = JointState()
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    joint_temp.header = h

    # for i in range(1,8):
    #     name = "iiwa_joint_" + str(i)
    #     joint_temp.name.append(name)
    #     if i == 4:
    #         joint_temp.position.append(1.57)
    #     elif i == 6:
    #         joint_temp.position.append(-1.57)
    #     else:
    #         joint_temp.position.append(0)
    #
    #     joint_temp.velocity.append(0)
    #
    # pub.publish(joint_temp)
    # rate.sleep()
rospy.spin()
