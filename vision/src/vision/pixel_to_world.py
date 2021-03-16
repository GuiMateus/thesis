#!/usr/bin/env python2

import tf
import rospy
import numpy as np

from service.srv import vision_detect, vision_detectResponse, robot_request, robot_requestResponse
from sensor_msgs.msg import PointCloud2, PointCloud
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker


class pixel_to_world():

    def __init__(self):
        self.toFrame = '/iiwa_link_0'
        self.fromFrame = '/camera_color_optical_frame'
        self.seq = 0

    def get_point(self, x, y):
        point = np.array([x, y, 0])
        msg = rospy.wait_for_message(
            "/camera/depth_registered/points", PointCloud2)
        depth = pc2.read_points(msg, field_names=(
            "x", "y", "z"), skip_nans=True, uvs=[(x, y)])  # Questionable
        cam_point = list(depth)
        break1 = False
        if(cam_point == []):
            for x_point in range(-5, 5):
                for y_point in range(-5, 5):
                    tempx = x + x_point
                    tempy = y + y_point
                    depth = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True, uvs=[
                                            (tempx, tempy)])  # Questionable
                    cam_point = list(depth)
                    # print cam_point
                    if(cam_point != []):
                        break1 = True
                        break
                if break1:
                    break

        if(cam_point != []):
            point = np.array(
                [cam_point[0][0], cam_point[0][1], cam_point[0][2]])
            print("point: ", cam_point)
            # publish_object_location(point)

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

    def robotRequestCB(self, req):

        worldPoints = robot_requestResponse()

        visionResults = rospy.ServiceProxy('vision_service', vision_detect)
        visionResponse = visionResults.call()
        pixelPoint = visionResponse.centroid

        pointCloudWorld = PointCloud()
        pointCloudWorld.header.stamp = rospy.Time.now()
        pointCloudWorld.header.frame_id = self.toFrame
        pointCloudWorld.header.seq = self.seq
        self.seq += 1

        if len(pixelPoint) > 0:

            for point in pixelPoint:
                worldPoint = self.get_point(int(point.x), int(point.y))
                temp = Point32()
                temp.x = worldPoint[0]
                temp.y = worldPoint[1]
                temp.z = worldPoint[2]
                pointCloudWorld.points.append(temp)
            worldPoints.centroid_world_coord = pointCloudWorld
            return worldPoints

        else:
            nullPoint = worldPoints
            temp = Point32()
            nullPoint.centroid_world_coord.points.append(temp)
            return nullPoint
            
def main():
    pw = pixel_to_world()
    rospy.init_node("pixel_to_world")
    rospy.Service("robotRequest", robot_request, pw.robotRequestCB)
    while(not rospy.is_shutdown()):
        rospy.spin()


if __name__ == "__main__":
    main()
