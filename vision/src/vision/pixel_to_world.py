#!/usr/bin/env python2

import tf
import rospy
import numpy as np
import cv2
import time
import json


from service.srv import vision_detect, vision_detectResponse, pixel2world_request, pixel2world_requestResponse
from sensor_msgs.msg import PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker
from pythonClasses.imageManipulation import imageManipulation


class pixel_to_world():

    def __init__(self):
        self.a = None

    def get_point(self, msg, x, y):

        depth = pc2.read_points(msg, field_names=(
            "x", "y", "z"), skip_nans=True, uvs=[
            (x, y)]) 
        cam_point = list(depth)
        break1 = False

        if(cam_point == []):
            for x_point in range(-5, 5):
                for y_point in range(-5, 5):
                    tempx = x + x_point
                    tempy = y + y_point
                    depth = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True, uvs=[
                                            (tempx, tempy)]) 
                    cam_point = list(depth)
                    if(cam_point != []):
                        break1 = True
                        break
                if break1:
                    break

        if(cam_point != []):
            point = np.array(
                [cam_point[0][0], cam_point[0][1], cam_point[0][2]])

        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform(
            self.toFrame, self.fromFrame, rospy.Time(), rospy.Duration(4))
        (trans, rot) = tf_listener.lookupTransform(
            self.toFrame, self.fromFrame, rospy.Time())

        world_to_cam = tf.transformations.compose_matrix(
            translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
        obj_vector = np.concatenate((point, np.ones(1))).reshape((4, 1))
        obj_base = np.dot(world_to_cam, obj_vector)
        print(obj_base[0:3])
        return cam_point

    def pixel2WorldRequestCB(self, req):
        
        msg = rospy.wait_for_message(
        "/camera/depth_registered/points", PointCloud2)

        pointSpace = self.get_point(msg, req.maskX.data, req.maskY.data)

        worldX, worldY, worldZ = Int32()

        worldX = pointSpace[0]
        worldY = pointSpace[1]
        worldZ = pointSpace[2]

        if worldX != None and worldY != None and worldZ != None:
            return worldX, worldY, worldZ
        else:
            return None, None, None


def main():
    pw = pixel_to_world()
    rospy.init_node("pixel_to_world")
    rospy.Service("pixel2world", pixel2world_request, pw.pixel2WorldRequestCB)
    while(not rospy.is_shutdown()):
        rospy.spin()


if __name__ == "__main__":
    main()
