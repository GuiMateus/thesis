#!/usr/bin/env python3

# import sys
import rospy
import cv2
import os
import time
import json
import numpy as np
import gc

from pythonClasses.pointCloudProcessing import pointCloudProcessing
from pythonClasses.acquireData import acquireImage
from pythonClasses.detectObjects import yoloInit
from pythonClasses.segmentationInit import segmentationInit
from pythonClasses.imageManipulation import imageManipulation
from pythonClasses.darknet import darknet
from service.srv import vision_detect, vision_detectResponse, setobject_request
from std_msgs.msg import String, Int32


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
        self.seq = 0
        self.pointCloudFileName = "/opt/vision/staticEnvironment/environmentCloud.ply"
        self.reconstructionType = "online"
        self.previousReconstructionType = ""
        self.dynamicObject = ""
        self.staticObject = ""

    def initializeYOLO(self):
        """Load YOLOv4 weights
        """
        # try:
        print(f"{bcolors.OKCYAN}Attempting to load{bcolors.ENDC} " + self.reconstructionType + f"{bcolors.OKCYAN} reconstruction object detector.{bcolors.ENDC}")
        yl = yoloInit()
        yl.reconstructionType = self.reconstructionType
        self.structure, self.classes, self.colour = yl.initialiseNetwork()
        print(f"{bcolors.OKGREEN}Object detector successfully loaded.{bcolors.ENDC}")
        # except:
        #     print(f"{bcolors.WARNING}An error occured while loading the object detector, are the paths to weights, model, and cfg correct?{bcolors.ENDC}")

    def initializeDeepLab(self):
        """Load DeepLabV3 weights
        """
        # try:
        print(
            f"{bcolors.OKCYAN} Now attempting to load{bcolors.ENDC} " + self.reconstructionType + f"{bcolors.OKCYAN} reconstruction segmentation model and weights.{bcolors.ENDC}")
        dl = segmentationInit()
        self.segModel = dl.deeplabInit(self.reconstructionType)
        print(f"{bcolors.OKGREEN}Segmentation model and weights loaded successfully, you can now use the models.{bcolors.ENDC}")
        # except:
        #     print(f"{bcolors.WARNING}An error occured whiel loading the segmentation weights and model, are the paths correct?{bcolors.ENDC}")

    def getImage(self):
        """Gets images from the rospy realsense sdk

        Returns:
            np.array: Incoming image
        """
        im = acquireImage()
        incomingImage = im.getROSImage()
        incomingDepth = im.getROSDepthImage()
        return incomingImage, incomingDepth

    def getStaticObject(self):
        if os.stat('.environmentReconstruction/ontologies.txt').st_size != 0:
            with open('.environmentReconstruction/ontologies.txt', 'r') as infile:
                ontologies = json.load(infile)

            for ontology in ontologies["Ontologies"]:
                if str(self.dynamicObject) == str(ontology['dynamicObject']):
                    self.staticObject = ontology['staticObject']
                    print(self.staticObject)

        

    def useYOLO(self, image):
        """Performs inference for object detection

        Args:
            image (np.array): Incoming image

        Returns:
            np.array, list: Drawn bounding boxes in image, List of strings with network detections
        """
        yl = yoloInit()
        yl.reconstructionType = self.reconstructionType
        yl.object = self.dynamicObject
        yl.staticObject = self.staticObject
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
        # dl.handleObjectCropping(segImage, detections, self.reconstructionType)
        # Convert np.arrays to PyTorch tensors
        tensorArray = dl.imageToTensor(segImage)

        self.i += 1
        # Segment all the cropped objects
        if len(tensorArray) > 0:
            masks = dl.inference(self.segModel, tensorArray,
                                 self.reconstructionType, segImage)
            # feedback, masksOutput = dl.toImgCoord(masks, depthImage, feedback)
            # crops = dl.getCrops()
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

        self.reconstructionType = req.reconstruction_type.data

        # Gather images and create copies to avoid memory address replacement
        incomingImage, incomingDepth = self.getImage()
        yoloImage = incomingImage.copy()
        segmentationImage = incomingImage.copy()
        feedBackImage = []

        cv2.imwrite(".environmentReconstruction/offlineReconstruction.png", incomingImage)
        
        if self.reconstructionType == "online":
            feedBackImage = cv2.imread(
                ".environmentReconstruction/offlineReconstruction.png")
            self.getStaticObject()

        elif self.reconstructionType == "offline":
            feedBackImage = segmentationImage
            darknet.free_network_ptr(self.structure)
            self.segModel = []
            gc.collect()
            self.initializeYOLO()
            self.initializeDeepLab()



        # Object detection
        # visualFeedbackObjects, detections = self.useYOLO(yoloImage)
        detections = None
        # Semantic segmentation
        visualFeedbackMasks, maskArray, crops = self.useDeepLab(
            segmentationImage, incomingDepth, detections, feedBackImage)

        if maskArray is not None and len(maskArray) != 0:
            stringMsg = String()
            detectionsMsg = vision_detectResponse()
            jstr = im.bbox2json(crops)
            stringMsg.data = jstr
            detectionsMsg.image_detections = stringMsg

            pp.pointCloudGenerate(visualFeedbackMasks, incomingDepth)
            pp.saveCloud(self.pointCloudFileName)

            if self.reconstructionType == "offline":
                cv2.imwrite(
                    ".environmentReconstruction/offlineReconstruction.png", visualFeedbackMasks)
                self.reconstructionType = "online"
                darknet.free_network_ptr(self.structure)
                self.segModel = []
                gc.collect()
                self.initializeYOLO()
                self.initializeDeepLab()

            cv2.imwrite(".environmentReconstruction/detections.png",
                        visualFeedbackObjects)
            cv2.imwrite(".environmentReconstruction/masks.png",
                        visualFeedbackMasks)
            return detectionsMsg

        else:
            print("No masks detected")
            return []

    def objectOfInterestServiceCalled(self, req):
        self.dynamicObject = req.setObject.data
        stopComplaining = Int32()
        stopComplaining.data = 0
        return stopComplaining


def main():
    vc = visionCentral()
    rospy.init_node("semanticSegmentationNode")
    rospy.Service("setObjectOfInterest", setobject_request,
                  vc.objectOfInterestServiceCalled)
    # Load online weights
    vc.initializeYOLO()
    vc.initializeDeepLab()

    rospy.Service("vision_service", vision_detect, vc.startService)
    while(not rospy.is_shutdown()):
        rospy.spin()


if __name__ == "__main__":
    main()
