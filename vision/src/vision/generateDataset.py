#!/usr/bin/env python3

import rospy
import cv2

from pythonClasses.acquireData import acquireImage
from main import visionCentral
# from pythonClasses.segmentationInit import segmentationInit


class generateDataset():
    def __init__(self):
        self.incomingImage = []

    def showImages(self):
        # self.incomingImage = cv2.imread("/home/gui/Documents/git/Yolo_mark/x64/Release/data/pumps-v2/datapoint" + str(i) + ".jpg")
        ai = acquireImage()
        self.incomingImage = ai.getROSImage()
        cv2.imshow("image", self.incomingImage)
        cv2.waitKey(1)

    def saveImages(self, index, filePath, image):
        number = str(index)
        cv2.imwrite(filePath + number + ".jpg", image)
        print("Image saved!!")


def main():
    gd = generateDataset()
    # vc = visionCentral()
    # dl = segmentationInit()
    rospy.init_node("generateDataset")
    # vc.initializeYOLO()
    i = 360
    # j = 0
    while not rospy.is_shutdown():
        gd.showImages()
        if(gd.incomingImage is not None):
            # tempImage = gd.incomingImage.copy()
            # _, detections = vc.useYOLO(gd.incomingImage)
            # images = dl.handleObjectCropping(tempImage, detections)
            
            if input(''):
            # for image in images:
                gd.saveImages(
                    i, "/home/gui/Documents/data/environment/dataNew/datapoint", gd.incomingImage)
                # j += 1
                i += 1



if __name__ == "__main__":
    main()
