import numpy as np

class imageManipulation():
    """Class containing utilities for image processing
    """

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