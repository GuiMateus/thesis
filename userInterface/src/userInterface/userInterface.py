#!/usr/bin/env python3

import rospy
import PySimpleGUI as sg
import cv2
import os
from std_msgs.msg import String
from service.srv import vision_detect
from vision.pythonClasses.imageManipulation import imageManipulation


class userInterface():

    def __init__(self):
        a = 0

    def interfaceCallback(self):
        # First the window layout in 2 column
        # Fr now will only show the name of the file that was chosen
        detector_column = [
                [sg.Text("Object detections")],
                [sg.Image(key="-DETECTOR-")],
        ]
        segment_column = [
            [sg.Text("Object segmentations")],
            [sg.Image(key="-SEGMENT-")],
        ]
        cloud_column = [

            [sg.Text("3D Reconstruction")],
            [sg.Image(key="-RECONSTRUCT-")],

        ]
        # ----- Full layout -----
        layout = [
            [ sg.Button('Offline Reconstruction')
            ],
            [sg.TabGroup([[
                sg.Tab('Object Detection', detector_column, tooltip = 'Shows the objects detected'),
                sg.Tab('Object Segmentation', segment_column, tooltip = 'Shows the object masks'),
                sg.Tab('3D Reconstruction', cloud_column, tooltip = 'Shows the 3D reconstruction of the environment')
            ]])
            ]
        ]
        window = sg.Window("3D Reconstruction GUI", layout)
        # Run the Event Loop    

        while True:
            event, values = window.read(timeout=1)
            if event == "Exit" or event == sg.WIN_CLOSED:
                break
            if event == "Offline Reconstruction":
                self.callOfflineService()
            # Folder name was filled in, make a list of files in the folder
            try:
                window["-DETECTOR-"].update(
                    filename="/home/gui/.environmentReconstruction/detections.png")
                window["-SEGMENT-"].update(
                    filename="/home/gui/.environmentReconstruction/masks.png")
                window["-RECONSTRUCT-"].update(
                    filename="/home/gui/.environmentReconstruction/cloud.png")

            except:
                pass
    
    def callOfflineService(self):
        os.system("rosservice call /robotRequest '{reconstruction_type: {data: offline}}'")
    


def main():
    rospy.init_node('userInterface')
    ui = userInterface()

    while not rospy.is_shutdown():
        ui.interfaceCallback()
        rospy.spin()


if __name__ == "__main__":
    main()
