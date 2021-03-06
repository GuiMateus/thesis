#!/usr/bin/env python3

import rospy
import cv2

from pythonClasses.acquireData import acquireImage


class generateDataset():
    def __init__(self):
        self.incomingImage = []

    def showImages(self):
        ai = acquireImage()
        self.incomingImage = ai.returnImage()
        cv2.imshow("image", self.incomingImage)
        cv2.waitKey(1)

    def saveImages(self, index, filePath):
        number = str(index)
        cv2.imwrite(filePath + number + ".jpg", self.incomingImage)
        print("Image saved!!")


def main():
    gd = generateDataset()
    rospy.init_node("generateDataset")
    i = 0
    while not rospy.is_shutdown():
        gd.showImages()
        if input(''):
            i = i + 1
            gd.saveImages(
                i, "/home/gui/Documents/P9_ws/src/rob9/vision/data/datapoint")


if __name__ == "__main__":
    main()
