#!/usr/bin/env python3

import rospy
import json
import cv2
import os
from std_msgs.msg import String, Int32
from service.srv import ontologies_request
from pathlib import Path


class ontologies():

    def __init__(self):
        temp = 0

    def ontologyServiceCalled(self, req):

        ontologies = []
        newOntology = True

        if os.stat('.environmentReconstruction/ontologies.txt').st_size != 0:
            with open('.environmentReconstruction/ontologies.txt', 'r') as infile:
                ontologies = json.load(infile)

            for ontology in ontologies["Ontologies"]:
                if str(req.dynamicObject.data) == str(ontology['dynamicObject']):
                    ontology['staticObject'] = req.staticObject.data
                    newOntology = False
        
            if newOntology == True:
                ontologies["Ontologies"].append({
                    'dynamicObject' : req.dynamicObject.data,
                    'staticObject' : req.staticObject.data
                })
            
            with open('.environmentReconstruction/ontologies.txt', 'w') as outfile:
                outfile.truncate(0)
                json.dump(ontologies, outfile)
        
        else:
            generateOntolgies = {}
            generateOntolgies['Ontologies'] = []
            generateOntolgies["Ontologies"].append({
                'dynamicObject' : req.dynamicObject.data,
                'staticObject' : req.staticObject.data
            })
            with open('.environmentReconstruction/ontologies.txt', 'w') as outfile:
                json.dump(generateOntolgies, outfile)

        test = Int32()
        test.data = 0
        return test


def main():
    rospy.init_node('ontologies')
    og = ontologies()
    rospy.Service("ontologiesRequest", ontologies_request, og.ontologyServiceCalled)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
