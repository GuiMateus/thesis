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
from std_msgs.msg import String, Int32, Float32
from visualization_msgs.msg import Marker
from pythonClasses.imageManipulation import imageManipulation


class pixel_to_world():

    def __init__(self):
        """Class constructor
        """
        self.toFrame = '/camera_color_optical_frame'
        self.fromFrame = '/camera_color_optical_frame'
        self.seq = 0

    def get_point(self, msg, u, v):
        """Gets an (x,y,z) point in world coordinates from (u,v)

        Args:
            msg (Point Cloud): Incoming Point Cloud from realsense camera
            u (float): [description]
            v ([type]): [description]

        Returns:
            [type]: [description]
        """
        u = int(u)
        v = int(v)
        depth = pc2.read_points(msg, field_names=(
            "x", "y", "z"), skip_nans=True, uvs=[
            (u, v)]) 
        cam_point = list(depth)
        break1 = False

        if(cam_point == []):
            for u_point in range(-5, 5):
                for v_point in range(-5, 5):
                    tempu = u + u_point
                    tempv = v + v_point
                    depth = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True, uvs=[
                                            (tempu, tempv)]) 
                    cam_point = list(depth)
                    if(cam_point != []):
                        break1 = True
                        break
                if break1:
                    break
        
        point = []
        if(cam_point != []):
            point = np.array([cam_point[0][0], cam_point[0][1], cam_point[0][2]])

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
        return obj_base

    def pixel2WorldRequestCB(self, req):
        """Pixel2world callback function

        Args:
            req (Ros service): Service call request

        Returns:
            [int]: (x,y,z) point
        """
        
        msg = rospy.wait_for_message(
        "/camera/depth_registered/points", PointCloud2)

        pointSpace = self.get_point(msg, req.x.data, req.y.data)

        worldX = Float32()
        worldY = Float32()
        worldZ = Float32()

        worldX.data = pointSpace[0]
        worldY.data = pointSpace[1]
        worldZ.data = pointSpace[2]

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
