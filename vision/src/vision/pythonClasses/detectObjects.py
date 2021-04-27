import cv2
import numpy as np
from .darknet import darknet
from .imageManipulation import imageManipulation


class yoloInit():
    """Class interfaces with YOLO and handles conversions to Darknet data members
    """

    def __init__(self):
        """Class constructor for yoloInit()
        """
        self.detections = []

    def initialiseNetwork(self):
        """Loads the network model, weights, and classes

        Returns:
            (model): Network model used  for inference
            (str list): Class names
            (int list): Classes colours
        """
        darknet.set_gpu(0)
        net, names, colours = darknet.load_network("/opt/vision/yoloConfig/dynamicEnvironmnet.cfg",
                                                   "/opt/vision/yoloConfig/dynamicEnvironment.data", "/opt/vision/weights/yolov4/yolov4DynamicEnvironment.weights", batch_size=1)
        return net, names, colours

    def array_to_image(self, cvImage, networkStructure, channels=3):
        """Converts an numpy array into an IMAGE c_struct from darknet

        Args:
            cvImage (np.array): Input image
            networkStructure (model): Network used for inference
            channels (int, optional): Number of channels in input image. Defaults to 3.

        Returns:
            (IMAGE): Darknet image
        """
        width, height = self.getNetworkDims(networkStructure)
        # Resize image to have the dimensions the NN expects
        cvImage = cv2.resize(cvImage, (width, height),
                             interpolation=cv2.INTER_LINEAR)
        # Transform the image from a np.array to a darknet.IMAGE type
        cvImage = cvImage.transpose(2, 0, 1)
        flat_image = np.ascontiguousarray(cvImage.flat, dtype=np.float32)/255.0
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

        # Project the coordinates found using the NN to the original image space
        self.detections = self.originalProjeciton(
            self.detections, networkStructure, cvImage)

        darknet.print_detections(self.detections)

        # If objects are found, they can be drawn
        if self.detections is not None:
            darknet.draw_boxes(self.detections, cvImage, colours)
            return cvImage, self.detections
        else:
            return None, None

    def originalProjeciton(self, detections, networkStructure, image):
        """Project pixel coordinates from Darknet image dims to original image dims

        Args:
            detections (list): Detections information
            networkStructure (model): Network model and weights
            image (np.array): Original image

        Returns:
            (list): List of detections updated to contain pixel coordinates of original image space
        """
        # Constant value for the minimum accepted confidence value for detections
        CONFIDENCEVALUE = 75
        detectionsOriginal = []

        im = imageManipulation()

        # Get the input size of the network
        networkWidth, networkHeight = self.getNetworkDims(networkStructure)
        
        # Get the size of the original input image
        imageHeight = image.shape[0]
        imageWidth = image.shape[1]

        # Find the image ratio 
        ratio = imageHeight/imageWidth

        # Convert all the detections to original image dimensions
        for label, confidence, bbox in detections:
            
            # Extracts bounding box data from struct
            projectedX, projectedY, projectedHeight, projectedWidth = im.extractBbox(
                bbox)

            if float(confidence) > CONFIDENCEVALUE:
                
                # Remove outliers
                if projectedX > 0 and projectedX < 10000 and projectedY > 0 and projectedY < 10000 and projectedWidth > 0 and projectedWidth < 10000 and projectedHeight > 0 and projectedHeight < 10000:

                    # Convert bounding boxes to percentages
                    percentageX = im.toPercent(projectedX, networkWidth)
                    percentageY = im.toPercent(projectedY, networkHeight)
                    percentageHeight = im.toPercent(projectedHeight, networkHeight)
                    percentageWidth = im.toPercent(projectedWidth, networkWidth)

                    # Convert percentages to original image pixel values and fix width/height ratio
                    originalX = im.toPixel(percentageX, imageWidth)
                    originalY = im.toPixel(percentageY, imageHeight)
                    originalWidth = im.toPixel(percentageWidth, imageWidth) * ratio
                    originalHeight = im.toPixel(percentageHeight, imageHeight) / ratio

                    # The dimensions from the darknet side seem to be flipped, so height and width must be swapped
                    bbox = im.compressBbox(originalX, originalY,
                                        originalHeight, originalWidth)
                    detectionsOriginal.append((label, confidence, (bbox)))
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
