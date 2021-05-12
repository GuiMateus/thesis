import cv2
import numpy as np
import os
import json
from .darknet import darknet
from .imageManipulation import imageManipulation


class yoloInit():
    """Class interfaces with YOLO and handles conversions to Darknet data members
    """

    def __init__(self):
        """Class constructor for yoloInit()
        """
        self.detections = []
        self.reconstructionType = ""
        self.dynamicObject = ""
        self.staticObject = ""
        self.staticObjectMinCoord = []
        self.cropRegionWidth = -1
        self.cropRegionHeight = -1
        self.cropRegionX = -1
        self.cropRegionY = -1


    def initialiseNetwork(self):
        """Loads the network model, weights, and classes

        Returns:
            (model): Network model used  for inference
            (str list): Class names
            (int list): Classes colours
        """
        if self.reconstructionType == "online":
            net, names, colours = darknet.load_network("/opt/vision/yoloConfig/dynamicEnvironment.cfg",
                                                       "/opt/vision/yoloConfig/dynamicEnvironment.data", "/opt/vision/weights/yolov4/yolov4DynamicEnvironment.weights", self.reconstructionType, batch_size=1)
            return net, names, colours

        elif self.reconstructionType == "offline":
            net, names, colours = darknet.load_network("/opt/vision/yoloConfig/staticEnvironment.cfg",
                                                       "/opt/vision/yoloConfig/staticEnvironment.data", "/opt/vision/weights/yolov4/yolov4StaticEnvironment.weights", self.reconstructionType, batch_size=1)
            return net, names, colours

        else:
            print("Error in type of reconstruction stated.")

    def array_to_image(self, cvImage, networkStructure, channels=3):
        """Converts an numpy array into an IMAGE c_struct from darknet

        Args:
            cvImage (np.array): Input image
            networkStructure (model): Network used for inference
            channels (int, optional): Number of channels in input image. Defaults to 3.

        Returns:
            (IMAGE): Darknet image
        """

        inputImage = []

        print(self.reconstructionType)
        if self.reconstructionType == "online":
            im = imageManipulation()
            offlineDetections = []
            minX = -1
            minY = -1
            maxX = -1
            maxY = -1

            if os.stat('.environmentReconstruction/predictions.json').st_size != 0:
                with open('.environmentReconstruction/predictions.json', 'r') as infile:
                    offlineDetections = json.load(infile)
                    print(offlineDetections)
                    print(offlineDetections["detections"])
                for detection in offlineDetections["detections"]:

                    if str(self.staticObject) == str(detection['label']):
                        minX = detection['minX']
                        minY = detection['minY']
                        maxX = detection['maxX']
                        maxY = detection['maxY']
                        print(min)
                        break
                    else:
                        print("CHOOSE AN OBJECT")
                        return None
            
            if minX != -1 and minY != -1 and maxX != -1 and maxY != -1: 
                self.cropRegionWidth = float(maxX) - float(minX)
                self.cropRegionHeight = float(maxY) - float(minY)
                self.cropRegionX = int(float(minX))
                self.cropRegionY = int(float(minY))

                inputImage = im.cropImage(cvImage, self.cropRegionX, self.cropRegionY, self.cropRegionWidth, self.cropRegionHeight)
                inputImage = inputImage[0]
                self.staticObjectMinCoord = [float(minX), float(minY)]
                    
        elif self.reconstructionType == "offline":
            inputImage = cvImage

        width, height = self.getNetworkDims(networkStructure)
        # Resize image to have the dimensions the NN expects
        inputImage = cv2.resize(inputImage, (width, height),
                             interpolation=cv2.INTER_LINEAR)
        # Transform the image from a np.array to a darknet.IMAGE type
        inputImage = inputImage.transpose(2, 0, 1)
        flat_image = np.ascontiguousarray(inputImage.flat, dtype=np.float32)/255.0
        darknetImageData = flat_image.ctypes.data_as(
            darknet.POINTER(darknet.c_float))
        darknetImage = darknet.IMAGE(width, height, channels, darknetImageData)
        return darknetImage

    def useDetection(self, darknetImage, networkStructure, classNames, cvImage, colours):
        """Detect objects

        Args:
            darknetImage (IMAGE): Darknet image
            networkStructure (model): Network model and weights
            classNames (str list): Names of classses
            cvImage (np.array): Original image
            colours (int list): Class colours

        Returns:
            (np.array): Image with drawn bounding boxes
            (list): Detections information
        """
        # Perform object detection
        self.detections = darknet.detect_image(
            networkStructure, classNames, darknetImage)

        print(cvImage.shape)

        # Project the coordinates found using the NN to the original image space
        self.detections = self.originalProjection(
            self.detections, networkStructure, cvImage)
        darknet.print_detections(self.detections)

        # If objects are found, they can be drawn
        if self.detections is not None:
            darknet.draw_boxes(self.detections, cvImage, colours)
            cv2.imwrite("/home/gui/plzwork.png", cvImage)
            return cvImage, self.detections
        else:
            return None, None

    def originalProjection(self, detections, networkStructure, image):
        """Project pixel coordinates from Darknet image dims to original image dims

        Args:
            detections (list): Detections information
            networkStructure (model): Network model and weights
            image (np.array): Original image

        Returns:
            (list): List of detections updated to containpixel coordinates of original image space
        """
        # Constant value for the minimum accepted confidence value for detections
        CONFIDENCEVALUE = 75
        detectionsOriginal = []

        jsonObject = {}
        jsonObject['detections'] = []

        im = imageManipulation()

        originalX, originalY, originalWidth, originalHeight = -1, -1, -1, -1

        # Get the input size of the network
        networkWidth, networkHeight = self.getNetworkDims(networkStructure)

        # Get the size of the original input image




        # Convert all the detections to original image dimensions
        for label, confidence, bbox in detections:

            # Extracts bounding box data from struct
            projectedX, projectedY, projectedHeight, projectedWidth = im.extractBbox(
                bbox)

            if float(confidence) > CONFIDENCEVALUE:

                # Remove outliers
                if projectedX > 0 and projectedX < 10000 and projectedY > 0 and projectedY < 10000 and projectedWidth > 0 and projectedWidth < 10000 and projectedHeight > 0 and projectedHeight < 10000:

                    # Convert bounding boxes to percentages
                    if self.reconstructionType == "online":

                        imageHeight = self.cropRegionHeight
                        imageWidth = self.cropRegionWidth
                        cropRegionX, cropRegionY, cropRegionWidth, cropRegionHeight = im.projectImage(projectedX, projectedY, networkWidth, networkHeight, projectedWidth, projectedHeight, imageWidth, imageHeight)
                        # originalImageHeight = image.shape[0]
                        # originalImageWidth = image.shape[1]
                        # originalX, originalY, originalWidth, originalHeight = im.projectImage(cropRegionX, cropRegionY, imageWidth, imageHeight, cropRegionWidth, cropRegionHeight, originalImageWidth, originalImageHeight)

                        originalWidth = cropRegionWidth
                        originalHeight = cropRegionHeight
                        originalX = cropRegionX + self.cropRegionX
                        originalY = cropRegionY + self.cropRegionY

                        # originalX = originalX + self.staticObjectMinCoord[0]
                        # originalY = originalY + self.staticObjectMinCoord[1]
                        

                    elif self.reconstructionType == "offline":
                        imageHeight = image.shape[0]
                        imageWidth = image.shape[1]
                        originalX, originalY, originalWidth, originalHeight = im.projectImage(projectedX, projectedY, networkWidth, networkHeight, projectedWidth, projectedHeight, imageWidth, imageHeight)
                        

                    # The dimensions from the darknet side seem to be flipped, so height and width must be swapped
                    bbox = im.compressBbox(originalX, originalY,
                                           originalHeight, originalWidth)
                    detectionsOriginal.append((label, confidence, (bbox)))

                    # Save detections into JSON file if offline reconstruction
                    if self.reconstructionType == "offline":
                        minx = originalX-originalWidth/2
                        miny = originalY-originalHeight/2
                        maxx = originalX+originalWidth/2
                        maxy = originalY+originalHeight/2
                        if minx < 0:
                            minx = 0
                        if maxx > imageWidth:
                            maxx = imageWidth
                        if miny < 0:
                            miny = 0
                        if maxy > imageHeight:
                            maxy = imageHeight

                        jsonObject['detections'].append({
                            'label': label,
                            'confidence': confidence,
                            'minX': str(minx),
                            'minY': str(miny),
                            'maxX': str(maxx),
                            'maxY': str(maxy)
                        })

        # Push the offline detections into JSON
        if self.reconstructionType == "offline" and jsonObject != {} and detections != None:
            with open('.environmentReconstruction/predictions.json', 'w') as outfile:
                json.dump(jsonObject, outfile)
        return detectionsOriginal

    def getNetworkDims(self, networkStructure):
        """Gets the dimensions of the images used on a network

        Args:
            networkStructure (model): Network model and weights

        Returns:
            (int): Width of the image
            (int): Height of the image
        """
        width = darknet.network_width(networkStructure)
        height = darknet.network_height(networkStructure)
        return width, height