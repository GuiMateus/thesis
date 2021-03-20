import numpy as np
import cv2
import rospy
import pickle
import json
import base64
# from pypcd import pypcd
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2, PointField


class imageManipulation():
    """Class containing utilities for image processing
    """
    
    def __init__(self):
        self.type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
        self.nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in self.type_mappings)
    
    def toPercent(self, coordinate, dimensionSize):
        """Converts a pixel coordinate to a percentage 
        corresponding to an image size

        Args:
            coordinate (float): Value in pixel space
            dimensionSize (int): Size of a dimension of the image (width or height)

        Returns:
            (float): Percentage value corresponding to the pixel position in an image
        """
        percentage = coordinate/dimensionSize
        return percentage

    def toPixel(self, percentage, dimensionSize):
        """Converts an image percentage to a pixel coordinate 

        Args:
            percentage (float): Percentage value
            dimensionSize (int): Size of a dimension of the image (width or height)

        Returns:
            (float): Pixel value corresponding to the percentage
        """
        pixel = percentage * dimensionSize
        return pixel

    def cropImage(self, inputImage, x, y, width, height):
        """Crops an image

        Args:
            inputImage (np.array): [Original image]
            x (float): Upper left x of cropping region
            y (float): Upper left y of cropping region
            width (float): Width of cropping region
            height (float): Height of cropping region

        Returns:
            (np.array): Cropped segment of the image
        """


        width = int(width)
        height = int(height)
        sumY = y+width
        sumX = x+height

        if x < 0:
            width = width + x
            x = 0
        if y < 0:
            height = y + height
            y = 0
        if sumY > inputImage.shape[0]:
            sumY = inputImage.shape[0]
        if sumX > inputImage.shape[1]:
            sumX = inputImage.shape[1]

        croppedImage = inputImage[y:y+height, x:x+width]
        return croppedImage

    def extractBbox(self, bbox):
        """Extracts x, y, w, h coordinates from a bbox method

        Args:
            bbox (list): Contains x, y, w, h

        Returns:
            (float): Values of x, y, w, h
        """
        x, y, w, h = bbox
        return x, y, w, h

    def compressBbox(self, x, y, w, h):
        """Compresses x, y, w, h of a bounding box into a bbox method, making value parsing easier

        Args:
            x (float): x coordinate of the center of a bounding box
            y (float): y coordinate of the center of a bounding box
            w (float): width of a bounding box
            h (float): height of a bounding box

        Returns:
            (list): Contains x, y, w, h
        """
        bbox = (x, y, w, h)
        return bbox

    def getImageShape(self, image):
        x = image.shape[0]
        y = image.shape[1]

        return x, y

    def im2json(self,im):
        """Convert a Numpy array to JSON string"""
        imdata = pickle.dumps(im, protocol=2)
        jstr = json.dumps({"image": base64.b64encode(imdata).decode('ascii')})
        return jstr

    def json2im(self,jstr):
        """Convert a JSON string back to a Numpy array"""
        load = json.loads(jstr)
        imdata = base64.b64decode(load['image'])
        im = pickle.loads(imdata)
        return im
