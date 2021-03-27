#!/usr/bin/env python3

# import sys
import rospy
import cv2
import time
import json
import numpy as np

from pythonClasses.pointCloudProcessing import pointCloudProcessing
from pythonClasses.acquireData import acquireImage
from pythonClasses.detectObjects import yoloInit
from pythonClasses.segmentationInit import segmentationInit
from pythonClasses.imageManipulation import imageManipulation
from service.srv import vision_detect, vision_detectResponse
from std_msgs.msg import String

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class visionCentral():
    """
    Central vision node which handles YOLOv4 and DeeplabV3 wrappers
    """

    def __init__(self):
        """visionCentral constructor
        """
        self.structure = []
        self.classes = []
        self.colour = []
        self.i = 0
        self.timeCount = 0
        self.segModel = []
        self.imageNorm = []
        self.toFrame = '/camera_color_optical_frame'
        self.fromFrame = '/camera_color_optical_frame'
        self.seq = 0
        self.pointCloudFileName = "/opt/vision/staticEnvironment/environmentCloud.ply"

    def initializeYOLO(self):
        """Load YOLOv4 weights
        """
        try:
            print(f"{bcolors.OKCYAN}Attempting to load object detector.{bcolors.ENDC}")
            yl = yoloInit()
            self.structure, self.classes, self.colour = yl.initialiseNetwork()
            print(f"{bcolors.OKGREEN}Object detector successfully loaded.{bcolors.ENDC}")
        except:
            print(f"{bcolors.WARNING}An error occured while loading the object detector, are the paths to weights, model, and cfg correct?{bcolors.ENDC}")

    def initializeDeepLab(self):
        """Load DeepLabV3 weights
        """
        try:
            print(f"{bcolors.OKCYAN} Now attempting to load segmentation model and weights.{bcolors.ENDC}")
            dl = segmentationInit()
            self.segModel, self.imageNorm = dl.deeplabInit()
            print(f"{bcolors.OKGREEN}Segmentation model and weights loaded successfully, you can now use the models.{bcolors.ENDC}")
        except:
            print(f"{bcolors.WARNING}An error occured whiel loading the segmentation weights and model, are the paths correct?{bcolors.ENDC}")

    def getImage(self):
        """Gets images from the rospy realsense sdk

        Returns:
            np.array: Incoming image
        """
        im = acquireImage()
        incomingImage = im.getROSImage()
        incomingDepth = im.getROSDepthImage()
        return incomingImage, incomingDepth

    def useYOLO(self, image):
        """Performs inference for object detection

        Args:
            image (np.array): Incoming image

        Returns:
            np.array, list: Drawn bounding boxes in image, List of strings with network detections
        """
        yl = yoloInit()
        # Convert np.array to darknet IMAGE C struct
        darkNetImage = yl.array_to_image(image, self.structure)
        # Use IMAGE, network weights, and classes to perform object detection
        visualFeedbackObjects, objectsDetected = yl.useDetection(
            darkNetImage, self.structure, self.classes, image, self.colour)
        return visualFeedbackObjects, objectsDetected

    def useDeepLab(self, segImage, depthImage, detections, feedback):
        """Performs inference for semantic segmentation

        Args:
            segImage (np.array): Input image
            detections (list[]): List of strings of object detections
        """
        dl = segmentationInit()
        masksOutput = []
        # Crop input image into sub-regions based on the information from object detection
        dl.handleObjectCropping(segImage, detections)
        # Convert np.arrays to PyTorch tensors
        tensorArray = dl.imageToTensor(self.imageNorm)
        self.i += 1
        # Segment all the cropped objects
        if len(tensorArray) > 0:
            masks = dl.inference(self.segModel, tensorArray, self.i)
            feedback, masksOutput = dl.toImgCoord(masks, depthImage, feedback)
            crops = dl.getCrops()
            return feedback, masksOutput, crops
        else:
            return None, None, None

    def startService(self, req):
        """Starts the vision service

        Args:
            req (ROS request): ROS vision request

        Returns:
            geometry_msgs/Point[]: Bounding box centroids
        """

        im = imageManipulation()
        pp = pointCloudProcessing()

        # Gather images and create copies to avoid memory address replacement
        incomingImage, incomingDepth = self.getImage()
        yoloImage = incomingImage.copy()
        segmentationImage = incomingImage.copy()

        # Object detection
        visualFeedbackObjects, detections = self.useYOLO(yoloImage)

        # Semantic segmentation
        visualFeedbackObjects, maskArray, crops = self.useDeepLab(segmentationImage, incomingDepth, detections, visualFeedbackObjects)

        if maskArray is not None and len(maskArray) != 0:
            stringMsg = String()
            detectionsMsg = vision_detectResponse()
            jstr = im.bbox2json(crops)
            stringMsg.data = jstr
            detectionsMsg.image_detections = stringMsg

            pp.pointCloudGenerate(visualFeedbackObjects, incomingDepth)
            pp.saveCloud(self.pointCloudFileName)
            return detectionsMsg
    
        else:
            print("No masks detected")
            return []



def main():
    vc = visionCentral()
    rospy.init_node("semanticSegmentationNode")
    vc.initializeYOLO()
    vc.initializeDeepLab()

    rospy.Service("vision_service", vision_detect, vc.startService)
    while(not rospy.is_shutdown()):
        rospy.spin()


if __name__ == "__main__":
    main()
