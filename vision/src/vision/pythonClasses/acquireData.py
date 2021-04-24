import rospy
import numpy as np
from sensor_msgs.msg import Image
from . import img

class acquireImage():
    """Class to acquire images from ROS
    """
    def __init__(self):
        """Class constructor
        """
        self.rosImage = []
        self.cvImage = []
        self.cvDepthImage = []

    def getROSImage(self):
        """Wait for images
        """
        self.rosImage = rospy.wait_for_message("/camera/color/image_rect_color", Image, timeout=11)
        self.cvImage = img.image_to_numpy(self.rosImage)
        self.cvImage = img.bgr_to_rgb(self.cvImage)
        return self.cvImage


    def getROSDepthImage(self):
        """Wait for depth images
        """
        msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image, timeout=11)
        self.cvDepthImage = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width, -1)
        return self.cvDepthImage