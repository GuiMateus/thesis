#!/usr/bin/env python3

# import sys
import rospy
import cv2
import os
import time
import json
import numpy as np
import gc
import torch
import seaborn as sn
import pandas as pd
import matplotlib as plt

from tqdm import tqdm
from PIL import Image, ImageFile
from torchvision import transforms



from pythonClasses.pointCloudProcessing import pointCloudProcessing
from pythonClasses.acquireData import acquireImage
from pythonClasses.detectObjects import yoloInit
from pythonClasses.segmentationInit import segmentationInit
from pythonClasses.imageManipulation import imageManipulation
from pythonClasses.darknet import darknet
from pythonClasses.deeplab.utils.metrics import Evaluator
from pythonClasses.deeplab.dataloaders.datasets import cityscapes, coco, combine_dbs, pascal, sbd
from service.srv import vision_detect, vision_detectResponse, setobject_request
from torch.utils.data import DataLoader
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
        kwargs = {'num_workers': 1, 'pin_memory': True}
        self.val_set = coco.COCOSegmentation(split='val')
        # self.val_set = pascal.VOCSegmentation(split='val')
        self.val_loader = DataLoader(self.val_set, batch_size=1, shuffle=False, **kwargs)
        self.evaluator = Evaluator(21)


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
        dl.handleObjectCropping(segImage, detections, self.reconstructionType)

        # Convert np.arrays to PyTorch tensors
        tensorArray = dl.imageToTensor()


        self.i += 1
        # Segment all the cropped objects
        if len(tensorArray) > 0:
            masks = dl.inference(self.segModel, tensorArray,
                                 self.reconstructionType)
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

        self.evaluator.reset()
        tbar = tqdm(self.val_loader, desc='\r')
        print(type(self.val_loader))
        test_loss = 0.0
        for i, sample in enumerate(tbar):
            success = False
            j = 0

            if i > 5:
                break


            while not success and j < 1:
                image, target = sample['image'], sample['label']
                plz = torch.squeeze(image)
                # im = transforms.ToPILImage()(target).convert("L")
                # im = np.array(im)
                plz2 = transforms.ToPILImage()(plz).convert("RGB")
                plz2 = np.array(plz2)
                plz2 = cv2.cvtColor(plz2, cv2.COLOR_RGB2BGR)

                yoloImage = plz2
                deepImage = plz2
                feedBackImage = np.zeros(plz2.shape, np.uint8)

                self.reconstructionType = req.reconstruction_type.data 
                

                # Gather images and create copies to avoid memory address replacement
                # incomingImage, incomingDepth = self.getImage()
                # yoloImage = incomingImage.copy()
                # segmentationImage = incomingImage.copy()
                # feedBackImage = []

                # if self.reconstructionType == "online":
                #     feedBackImage = cv2.imread(
                #         ".environmentReconstruction/offlineReconstruction.png")
                #     self.getStaticObject()

                # elif self.reconstructionType == "offline":
                #     feedBackImage = segmentationImage
                #     darknet.free_network_ptr(self.structure)
                #     self.segModel = []
                #     gc.collect()
                #     self.initializeYOLO()
                #     self.initializeDeepLab()

                incomingDepth = None

                # Object detection
                visualFeedbackObjects, detections = self.useYOLO(yoloImage)

                # Semantic segmentation
                visualFeedbackMasks, maskArray, crops = self.useDeepLab(
                    deepImage, incomingDepth, detections, feedBackImage)

                
                
                if visualFeedbackMasks is not None:
                    visualFeedbackMasks = cv2.cvtColor(visualFeedbackMasks, cv2.COLOR_BGR2RGB)
                    image = Image.fromarray(visualFeedbackMasks.astype('uint8'), 'RGB')
                    tensorImage = transforms.ToTensor()(image).unsqueeze_(0)

                    # loss = self.criterion(tensorImage, target)
                    # test_loss += loss.item()
                    # tbar.set_description('Test loss: %.3f' % (test_loss / (i + 1)))
                    pred = tensorImage.data.cpu().numpy()
                    target = target.cpu().numpy()
                    pred = np.argmax(pred, axis=1)
                    # Add batch sample into evaluator
                    self.evaluator.add_batch(target, pred)
                    print("Iteration done!")
                    success = True

                else:
                    darknet.free_network_ptr(self.structure)
                    self.initializeYOLO()
                    j = j + 1
            


            # Fast test during the training
            Acc = self.evaluator.Pixel_Accuracy()
            Acc_class = self.evaluator.Pixel_Accuracy_Class()
            mean_accuracy = self.evaluator.Mean_Accuracy()
            mIoU = self.evaluator.Mean_Intersection_over_Union()
            FWIoU = self.evaluator.Frequency_Weighted_Intersection_over_Union()
            print("Acc:{}, Acc_class:{}, mAcc:{}, mIoU:{}, fwIoU: {}".format(Acc, Acc_class, mean_accuracy, mIoU, FWIoU))

        # self.writer.add_scalar('val/total_loss_epoch', test_loss, epoch)
        # self.writer.add_scalar('val/mIoU', mIoU, epoch)
        # self.writer.add_scalar('val/Acc', Acc, epoch)
        # self.writer.add_scalar('val/Acc_class', Acc_class, epoch)
        # self.writer.add_scalar('val/fwIoU', FWIoU, epoch)
        # print('Validation:')
        # print('[Epoch: %d, numImages: %5d]' % (epoch, i * self.args.batch_size + image.data.shape[0]))
        # print('Loss: %.3f' % test_loss)


            # if maskArray is not None and len(maskArray) != 0:
            #     stringMsg = String()
            #     detectionsMsg = vision_detectResponse()
            #     jstr = im.bbox2json(crops)
            #     stringMsg.data = jstr
            #     detectionsMsg.image_detections = stringMsg

                # pp.pointCloudGenerate(visualFeedbackMasks, incomingDepth)
                # pp.saveCloud(self.pointCloudFileName)

                # if self.reconstructionType == "offline":
                #     cv2.imwrite(
                #         ".environmentReconstruction/offlineReconstruction.png", visualFeedbackMasks)
                #     self.reconstructionType = "online"
                #     darknet.free_network_ptr(self.structure)
                #     self.segModel = []
                #     gc.collect()
                #     self.initializeYOLO()
                #     self.initializeDeepLab()

                # cv2.imwrite(".environmentReconstruction/detections.png",
                #             visualFeedbackObjects)
                # cv2.imwrite(".environmentReconstruction/masks.png",
                #             visualFeedbackMasks)
        return None

        # else:
        #     print("No masks detected")
        #     return []

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
