#!/usr/bin/env python3

import rospy
import PySimpleGUI as sg
import cv2
import os
from std_msgs.msg import String
from service.srv import vision_detect, robot_request
from vision.pythonClasses.imageManipulation import imageManipulation


class userInterface():

    def __init__(self):
        self.dynamicObjectClasses = []
        self.dynamicObjectClassesAll = []
        self.staticObjectClasses = []
        self.objectPicked = -1

    def interfaceCallback(self):
        # First the window layout in 2 column
        # Fr now will only show the name of the file that was chosen
        sg.theme('DarkAmber')
        detector_tab = [
                [sg.Text("Bounding boxes of objects detected in the environment.")],
                [sg.Image(key="-DETECTOR-")],
        ]
        segment_tab = [
            [sg.Text("Object masks detected in the environment.")],
            [sg.Image(key="-SEGMENT-")],
        ]
        cloud_tab = [

            [sg.Text("3D Pointcloud reconstruction of the environment.")],
            [sg.Image(key="-RECONSTRUCT-")],
        ]
        ontology = [
            [sg.Text("Assign dynamically detected objects with static object classes.")],
            # *[[sg.Text(objectClass, font='Courier 14'),] for objectClass in self.dynamicObjectClasses], 
            [sg.Listbox(list(self.dynamicObjectClasses), size=(30,20), enable_events=False, font='Courier 14'), sg.Listbox(list(self.staticObjectClasses), size=(30,20), enable_events=False, font='Courier 14')],
        ]
        
        # ----- Full layout -----
        layout = [
            [   sg.Button('Offline Reconstruction'),
                sg.Listbox(list(self.dynamicObjectClassesAll), size=(30,4), enable_events=False, tooltip='Specify which object should be included in online reconstruction. To include all objects, select "All".'),          
                sg.Button('Online Reconstruction'),
                sg.Button('Save Ontology'),
            ],
            [sg.TabGroup([[
                sg.Tab('Object Detection', detector_tab, tooltip = 'Shows the objects detected.'),
                sg.Tab('Object Segmentation', segment_tab, tooltip = 'Shows the object masks.'),
                sg.Tab('3D Reconstruction', cloud_tab, tooltip = 'Shows the 3D reconstruction of the environment.'),
                sg.Tab('Create ontology relations', ontology, tooltip = 'Create object ontologies between static and dynamic object classes.')
            ]])
            ]
        ]
        window = sg.Window("3D Reconstruction GUI", layout)
        # Run the Event Loop    

        while True:
            event, values = window.read(timeout=1)
            print(values)
            if event == "Exit" or event == sg.WIN_CLOSED:
                break
            if event == "Offline Reconstruction":
                self.callOfflineService()
            if event == "Online Reconstruction":
                self.callOnlineService()
            if event == "Save Ontology":
                print("plz")
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
        inputMessage = String()
        inputMessage.data = "offline"
        offlineResults = rospy.ServiceProxy('robotRequest', robot_request)
        visionResponse = offlineResults(inputMessage)

    def callOnlineService(self):
        os.system("rosservice call /robotRequest '{reconstruction_type: {data: online}}'")
    
    def getObjectClasses(self):
        with open('/opt/vision/yoloConfig/static.names', 'r') as file:
            content = file.readlines()
            for objectClass in content:
                fixedClass = objectClass.replace('\n', '')
                self.staticObjectClasses.append(fixedClass)
    
        with open('/opt/vision/yoloConfig/dynamic.names', 'r') as file:
            content = file.readlines()
            self.dynamicObjectClassesAll.append("All")
            for objectClass in content:
                fixedClass = objectClass.replace('\n', '')
                self.dynamicObjectClasses.append(fixedClass)
                self.dynamicObjectClassesAll.append(fixedClass)
    


def main():
    rospy.init_node('userInterface')
    ui = userInterface()
    ui.getObjectClasses()

    while not rospy.is_shutdown():
        ui.interfaceCallback()
        rospy.spin()


if __name__ == "__main__":
    main()
