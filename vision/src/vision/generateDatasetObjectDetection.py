#!/usr/bin/env python3

import rospy
import cv2

# from pythonClasses.visionCentral import visionCentral
from main import visionCentral
from pythonClasses.acquireData import acquireImage


class generateDataset():
    def __init__(self):
        self.incomingImage = []

    def showImages(self, i):
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
    rospy.init_node("generateDataset")
    i = 59
    while not rospy.is_shutdown():
        gd.showImages(i)
        if(gd.incomingImage is not None):
            if input(''):
                gd.saveImages(
                    i, "/home/gui/Documents/data/environment/YOLOStatic/datapoint", gd.incomingImage)
                i += 1



if __name__ == "__main__":
    main()
