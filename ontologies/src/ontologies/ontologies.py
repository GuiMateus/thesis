#!/usr/bin/env python3

import rospy
import cv2
import os
from owlready2 import *
from std_msgs.msg import String
from service.srv import ontologies_request
from pathlib import Path




class ontologies():

    def __init__(self):
        temp = 0

    def firstOntology(self):
        print("aaa")
        ontologyTemplate = owlready2.get_ontology("http://www.lesfleursdunormal.fr/static/_downloads/pizza_onto.owl").load()
        owlready2.onto_path.append("/home/gui/.environmentReconstruction")
        ontologyTemplate.save()

    




def main():
    rospy.init_node('ontologies')
    og = ontologies()
    ontologiesExist = Path("/home/gui/.environmentReconstruction/visionOntologies.rdfxml")
    if not ontologiesExist.is_file():
        og.firstOntology()
    rospy.Service("ontologies_service", ontologies_request, vc.startService)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
