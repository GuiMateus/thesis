#!/usr/bin/env python3

import rospy
import PySimpleGUI as sg
import cv2
import os
from std_msgs.msg import String
from service.srv import vision_detect, ontologies_request, setobject_request
from vision.pythonClasses.acquireData import acquireImage


class userInterface():
    """State machine used to create user interface and control the system's states
    """

    def __init__(self):
        self.dynamicObjectClasses = []
        self.dynamicObjectClassesAll = []
        self.staticObjectClasses = []
        self.objectPicked = -1
        self.AAULogo = []
        self.taskOntology = []

    def interfaceCallback(self):
        """Creates user interface
        """
        # First the window layout in 2 column
        # Fr now will only show the name of the file that was chosen
        sg.theme('DarkTeal12')

        live_tab = [
            [sg.Text(
                "Raw live camera feed.", font='Courier 14')],
            [sg.Image(key="-RAW-")],
        ]
        detector_tab = [
            [sg.Text(
                "Bounding boxes of objects detected in the environment.", font='Courier 14')],
            [sg.Image(key="-DETECTOR-")],
        ]
        segment_tab = [
            [sg.Text("Object masks detected in the environment.", font='Courier 14')],
            [sg.Image(key="-SEGMENT-")],
        ]
        cloud_tab = [

            [sg.Text("3D Pointcloud reconstruction of the environment.",
                     font='Courier 14')],
            [sg.Image(key="-RECONSTRUCT-")],
        ]
        ontology_tab = [
            [sg.Text(
                "Assign dynamically detected objects with static object classes and a task.", font='Courier 14')],
            [sg.Button('Save Ontology', font='Courier 14', button_color=('black', 'white'))],
            [sg.Listbox(list(self.dynamicObjectClasses), size=(20, 20), enable_events=False, font='Courier 14', pad=(100, 100), key="-STATIC-"), sg.Listbox(list(self.staticObjectClasses), size=(20, 20), enable_events=False, font='Courier 14', pad=(100, 100), key="-DYNAMIC-"), sg.Listbox(list(self.taskOntology), size=(20, 20), enable_events=False, font='Courier 14', pad=(100, 100), key="-TASK-")],
        ]
        selectObject_tab = [
            [sg.Text("Press image to find object.", font='Courier 14')],
            [
                sg.ReadFormButton('Pump', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/pump.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Pump"),
                sg.ReadFormButton('Cleaning bottle', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/cleaningBottle.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Cleaning Bottle"),
                sg.ReadFormButton('Saw', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/saw.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Saw"),
                sg.ReadFormButton('Box Cutter', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/boxCutter.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Box Cutter")
            ],

            [
                sg.ReadFormButton('Tape', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/tape.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Tape"),
                sg.ReadFormButton('Screwdriver', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/screwDriver.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Screwdriver"),
                sg.ReadFormButton('Wire cutter', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/wireCutter.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Wire Cutter"),
                sg.ReadFormButton('Tool box', button_color=('white'), font='Courier 14', image_filename="/opt/vision/GUIImages/toolBox.png",
                                  image_size=(255, 255), image_subsample=1, border_width=1, pad=(25, 50), key="Tool Box")
            ]
        ]

        # Use stuff from above in layout to create the interface and setup tabs
        layout = [
            [
                sg.Image(filename="/opt/vision/GUIImages/aau.png", pad=(75, 0)),
                sg.Button('Offline Reconstruction', pad=(50, 0),
                          font='Courier 14', button_color=('black', 'white')),
                sg.Button('Online Reconstruction', pad=(50, 0),
                          font='Courier 14', button_color=('black', 'white')),
                sg.Image(filename="/opt/vision/GUIImages/lhLogo.png",
                         pad=(75, 10))
            ],
            [
                sg.TabGroup([[
                    sg.Tab('Live Camera Feed', live_tab,
                           tooltip='Shows a live raw camera feed.'),
                    sg.Tab('Object Detection', detector_tab,
                           tooltip='Shows the objects detected.'),
                    sg.Tab('Object Segmentation', segment_tab,
                           tooltip='Shows the object masks.'),
                    sg.Tab('3D Reconstruction', cloud_tab,
                           tooltip='Shows the 3D reconstruction of the environment.'),
                    sg.Tab('Object selection', selectObject_tab,
                           tooltip='Press an object image in order to look for it.'),
                    sg.Tab('Create ontology relations', ontology_tab,
                           tooltip='Create object ontologies between static and dynamic object classes.'),



                ]], font='Courier 14')
            ]
        ]
        window = sg.Window("3D Reconstruction GUI", layout)
        rawFeed =  window["-RAW-"]

        # Run the Event Loop
        while True:
            event, values = window.read(timeout=1)

            # Check which event user triggers
            if event == "Exit" or event == sg.WIN_CLOSED:
                break
            if event == "Offline Reconstruction":
                self.callVisionService("offline")
            if event == "Online Reconstruction":
                self.callVisionService("online")
            if event == "Save Ontology":
                self.callObjectOntologies(
                    values["-STATIC-"], values["-DYNAMIC-"], values["-TASK-"])
                sg.Popup('Object Ontology Saved. '+values["-STATIC-"][0]+' located at '+ values["-DYNAMIC-"][0] + ', used in ' + values["-TASK-"][0]+'.', keep_on_top=True)
            if event == "Pump" or event == "Cleaning Bottle" or event == "Tape" or event == "Screwdriver" or event == "Toolbox" or event == "Box Cutter" or event == "Wire Cutter" or event == "Saw":
                self.setObjectOfInterest(event)
                sg.Popup('Dynamic Object ' + event + ' selected.', keep_on_top=True)


            # Folder name was filled in, make a list of files in the folder
            try:
                
                imgbytes = cv2.imencode('.png', self.getImage())[1].tobytes()
                rawFeed.update(data=imgbytes)
                window["-DETECTOR-"].update(
                    filename="/home/gui/.environmentReconstruction/detections.png")
                window["-SEGMENT-"].update(
                    filename="/home/gui/.environmentReconstruction/masks.png")
                window["-RECONSTRUCT-"].update(
                    filename="/home/gui/.environmentReconstruction/cloud.png")

            except:
                pass        

    def getImage(self):
        ai = acquireImage()
        rawImage = ai.getROSImage()
        return rawImage


    def setObjectOfInterest(self, object):
        """Set an object of interest for online detection

        Args:
            object (string): Dynamic object name
        """
        inputMessageObjectInterest = String()

        inputMessageObjectInterest.data = object

        setObjectService = rospy.ServiceProxy(
            'setObjectOfInterest', setobject_request)
        setObjectService(inputMessageObjectInterest)

    def callObjectOntologies(self, staticObject, dynamicObject, task):
        """Set an ontological relation

        Args:
            staticObject (string): Static object name
            dynamicObject (string): Dyanmic object name
            task (string): Task name
        """
        inputMessageStatic = String()
        inputMessageDynamic = String()
        inputMessageTask = String()

        inputMessageStatic.data = str(staticObject[0])
        inputMessageDynamic.data = str(dynamicObject[0])
        inputMessageTask.data = str(task[0])


        ontologiesService = rospy.ServiceProxy(
            'ontologiesRequest', ontologies_request)
        ontologiesService(inputMessageDynamic, inputMessageStatic, inputMessageTask)

    def callVisionService(self, message):
        """Call reconstruction service

        Args:
            message (string): Type of reconstruction
        """
        inputMessage = String()
        inputMessage.data = message
        visionService = rospy.ServiceProxy('vision_service', vision_detect)
        visionResponse = visionService(inputMessage)

    def getObjectClasses(self):
        """Load names of static and dynamic objects and tasks
        """
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
        
        with open('/opt/vision/GUIImages/taskOntologies.names', 'r', encoding='utf-8-sig') as file:
            content = file.readlines()
            for objectClass in content:
                fixedClass = objectClass.replace('\n', '')
                self.taskOntology.append(fixedClass)


def main():
    rospy.init_node('userInterface')
    ui = userInterface()
    ui.getObjectClasses()

    while not rospy.is_shutdown():
        ui.interfaceCallback()
        rospy.spin()


if __name__ == "__main__":
    main()
