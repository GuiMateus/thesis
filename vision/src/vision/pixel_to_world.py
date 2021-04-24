#!/usr/bin/env python2

import tf
import rospy
import numpy as np
import cv2
import time
import json



from service.srv import vision_detect, vision_detectResponse, robot_request, robot_requestResponse
from sensor_msgs.msg import PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from pythonClasses.imageManipulation import imageManipulation


class pixel_to_world():

    def __init__(self):
        self.toFrame = '/camera_color_optical_frame'
        self.fromFrame = '/camera_color_optical_frame'
        self.seq = 0

    def rgb2float(self, coloursRaw):
        colors = np.asarray(coloursRaw) * 255
        rgb = colors.astype(np.uint32)
        encoded_colors = np.array((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | (rgb[:, 2] << 0),
                    dtype=np.uint32)
        encoded_colors = np.cast[np.float32](encoded_colors)
        return encoded_colors

    def get_point(self, msg, x, y):
       
        depth = pc2.read_points(msg, field_names=(
            "x", "y", "z"), skip_nans=True, uvs=[
                                            (x, y)])  # Questionable
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
        return cam_point

    def robotRequestCB(self, req):

        im = imageManipulation()
        strMsg = String()
        worldPoints = robot_requestResponse()

        visionResults = rospy.ServiceProxy('vision_service', vision_detect)
        visionResponse = visionResults.call()

        strMsg = visionResponse.image_detections
        stringDetections = strMsg.data
        print(stringDetections)
        detections = im.json2bbox(stringDetections)
        # cv2.imwrite("/home/gui/aa.png", pixelPoint)
       
        msg = rospy.wait_for_message(
        "/camera/depth_registered/points", PointCloud2)

        pointCloudWorld = PointCloud()
        pointCloudWorld.header.stamp = rospy.Time.now()
        pointCloudWorld.header.frame_id = self.toFrame
        pointCloudWorld.header.seq = self.seq
        self.seq += 1
        
        pointArray = []

        if len(detections) > 0:

            jsonObject = {}
            jsonObject['detections'] = []

            for label, confidence, bbox in detections:
                xmin, ymin, xmax, ymax = im.extractBbox(bbox)
                bboxMinWorld = self.get_point(msg, int(xmin), int(ymin))
                bboxMaxWorld = self.get_point(msg, int(xmax), int(ymax))


                jsonObject['detections'].append({
                    'label' : label,
                    'confidence' : confidence,
                    'min' : str(bboxMinWorld),
                    'max' : str(bboxMaxWorld)
                    
                })
            with open('.environmentReconstruction/predictions.json', 'w') as outfile:
                json.dump(jsonObject, outfile)

            nullPoint = worldPoints
            temp = Point32()
            nullPoint.world_semantics.append(temp)
            print("Done!")
            return worldPoints

        else:
            nullPoint = worldPoints
            temp = Point32()
            nullPoint.world_semantics.points.append(temp)
            return nullPoint
            
def main():
    pw = pixel_to_world()
    rospy.init_node("pixel_to_world")
    rospy.Service("robotRequest", robot_request, pw.robotRequestCB)
    # pub = rospy.Publisher("pointCloudSemantics", PointCloud2)
    while(not rospy.is_shutdown()):
        # pw.robotRequestCB(1)
        # rospy.publish(pub)
        rospy.spin()


if __name__ == "__main__":
    main()
