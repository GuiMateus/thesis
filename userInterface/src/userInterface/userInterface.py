#!/usr/bin/env python3

import rospy
import PySimpleGUI as sg
import cv2
import os
from std_msgs.msg import String
from service.srv import vision_detect, robot_request, ontologies_request
from vision.pythonClasses.imageManipulation import imageManipulation


class userInterface():

    def __init__(self):
        self.dynamicObjectClasses = []
        self.dynamicObjectClassesAll = []
        self.staticObjectClasses = []
        self.objectPicked = -1
        self.AAULogo = []

    def interfaceCallback(self):
        # First the window layout in 2 column
        # Fr now will only show the name of the file that was chosen
        sg.theme('DarkTeal12')
        detector_tab = [
            [sg.Text("Bounding boxes of objects detected in the environment.", font='Courier 14')],
            [sg.Image(key="-DETECTOR-")],
        ]
        segment_tab = [
            [sg.Text("Object masks detected in the environment.", font='Courier 14')],
            [sg.Image(key="-SEGMENT-")],
        ]
        cloud_tab = [

            [sg.Text("3D Pointcloud reconstruction of the environment.", font='Courier 14')],
            [sg.Image(key="-RECONSTRUCT-")],
        ]
        ontology = [
            [sg.Text("Assign dynamically detected objects with static object classes.", font='Courier 14')],
            # *[[sg.Text(objectClass, font='Courier 14'),] for objectClass in self.dynamicObjectClasses], 
            [sg.Listbox(list(self.dynamicObjectClasses), size=(30,20), enable_events=False, font='Courier 14', pad=(100,100), key="-STATIC-"), sg.Button('Save Ontology', font='Courier 14', button_color=('black', 'white')), sg.Listbox(list(self.staticObjectClasses), size=(30,20), enable_events=False, font='Courier 14', pad=(100,100), key="-DYNAMIC-")],
        ]
        
        # ----- Full layout -----
        layout = [
            [   
                sg.Image(filename="/opt/vision/aau.png", pad=(75,0)),
                sg.Button('Offline Reconstruction', pad=(50,0), font='Courier 14', button_color=('black', 'white')),
                # sg.Listbox(list(self.dynamicObjectClassesAll), size=(30,4), enable_events=False, tooltip='Specify which object should be included in online reconstruction. To include all objects, select "All".'),          
                sg.Button('Online Reconstruction', pad=(50,0), font='Courier 14', button_color=('black', 'white')),
                sg.Image(filename="/opt/vision/lhLogo.png", pad=(75,10))
            ],
            [
                sg.TabGroup([[
                sg.Tab('Object Detection', detector_tab, tooltip = 'Shows the objects detected.'),
                sg.Tab('Object Segmentation', segment_tab, tooltip = 'Shows the object masks.'),
                sg.Tab('3D Reconstruction', cloud_tab, tooltip = 'Shows the 3D reconstruction of the environment.'),
                sg.Tab('Create ontology relations', ontology, tooltip = 'Create object ontologies between static and dynamic object classes.'),
                
            ]], font='Courier 14')
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
                self.callRobotService("offline")
            if event == "Online Reconstruction":
                self.callRobotService("online")
            if event == "Save Ontology":
                self.callObjectOntologies(values["-STATIC-"], values["-DYNAMIC-"])
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

    def callObjectOntologies(self, staticObject, dynamicObject):
        inputMessageStatic = String()
        inputMessageDynamic = String()

        inputMessageStatic.data = str(staticObject[0])
        inputMessageDynamic.data = str(dynamicObject[0])

        ontologiesService = rospy.ServiceProxy('ontologiesRequest', ontologies_request)
        ontologiesService(inputMessageDynamic, inputMessageStatic)


    def callRobotService(self, message):
        inputMessage = String()
        inputMessage.data = message
        robotService = rospy.ServiceProxy('robotRequest', robot_request)
        visionResponse = robotService(inputMessage)

    
    def getObjectClasses(self):
        with open('/opt/vision/yoloConfig/staticEnvironment.names', 'r', encoding='utf-8-sig') as file:
            content = file.readlines()
            for objectClass in content:
                fixedClass = objectClass.replace('\n', '')
                self.staticObjectClasses.append(fixedClass)
    
        with open('/opt/vision/yoloConfig/dynamicEnvironment.names', 'r', encoding='utf-8-sig') as file:
            content = file.readlines()
            for objectClass in content:
                fixedClass = objectClass.replace('\n', '')
                self.dynamicObjectClasses.append(fixedClass)
                self.dynamicObjectClassesAll.append(fixedClass)
    
    def getLogos(self):
        self.AAULogo = cv2.imread("/opt/vision/aau.png")

    


def main():
    rospy.init_node('userInterface')
    ui = userInterface()
    ui.getObjectClasses()

    while not rospy.is_shutdown():
        ui.interfaceCallback()
        rospy.spin()


if __name__ == "__main__":
    main()
