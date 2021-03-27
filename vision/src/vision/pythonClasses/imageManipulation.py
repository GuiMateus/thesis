import pickle
import json
import base64


class imageManipulation():
    """Class containing utilities for image processing
    """
    
    def __init__(self):
       a = 0
    
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


        if x < 0:
            width = width + x
            x = 0
        if y < 0:
            height = y + height
            y = 0

        sumY = y+height
        sumX = x+width

        if sumY > inputImage.shape[0]:
            sumY = inputImage.shape[0]
        if sumX > inputImage.shape[1]:
            sumX = inputImage.shape[1]

        croppedImage = inputImage[y:sumY, x:sumX]
        bbox = self.compressBbox(x, y, sumX, sumY)
        return croppedImage, bbox

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

    def bbox2json(self, detections):
        jsonArray = []
        for detection in detections:
            x, y, w, h = detection[3]

            buildJson = {
                "label": str(detection[1]),
                "confidence": str(detection[2]),
                "x": str(x),
                "y": str(y),
                "w": str(w),
                "h": str(h)
            }

                # convert into JSON:
            jsonObject = json.dumps(buildJson)
            jsonArray.append(jsonObject)
        finalJson = json.dumps(jsonArray)
        print(finalJson)
        return finalJson

    def json2bbox(self, jsonBbox):
        detectionsArray = []
        stringDetections = json.loads(jsonBbox)
        for detection in stringDetections:
            jsonDetection = json.loads(detection)
            label = jsonDetection["label"]
            confidence = jsonDetection["confidence"]
            bbox = self.compressBbox(jsonDetection["x"], jsonDetection["y"], jsonDetection["w"], jsonDetection["h"])

            detectionsArray.append([label, confidence, bbox])
        return detectionsArray


            
