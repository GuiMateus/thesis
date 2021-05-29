#!/usr/bin/env python

import rospy
from service.srv import pixel2world_request, pixel2world_requestResponse
from sensor_msgs.msg import PointCloud, JointState
from std_msgs.msg import Empty


def matlab_cb(msg):
    pointCloudWorld = PointCloud()
    visionClient = rospy.ServiceProxy("/pixel2world", pixel2world_request)
    visionResponse = visionClient.call()
    pointCloudWorld = visionResponse.centroid_world_coord
    publisher = rospy.Publisher("vision_positions_pc", PointCloud, queue_size=1)
    publisher.publish(pointCloudWorld)

def joint_cb(msg):
    global joint_states
    joint_states = msg

def main():
    rospy.init_node("central_com")
    rospy.Subscriber("matlab_trigger", Empty, matlab_cb)
    rospy.Subscriber("joint_temp", JointState, joint_cb)
    pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    global joint_states
    joint_states = JointState()

    for i in range(1,8):
        name = "iiwa_joint_" + str(i)
        joint_states.name.append(name)
        joint_states.position.append(0)
        joint_states.velocity.append(0)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        global joint_states
        joint_states.header.stamp = rospy.Time.now()

        pub.publish(joint_states)
        rate.sleep()






if __name__ == '__main__':
    main()
