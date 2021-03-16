import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from . import img
from . import imageManipulation

class acquireImage():
    """Class to acquire images from ROS
    """
    def __init__(self):
        """Class constructor
        """
        self.rosImage = []
        self.rosDepthImage = []
        self.cvImage = []
        self.cvDepthImage = []

    def getROSImage(self):
        """Wait for images
        """
        self.rosImage = rospy.wait_for_message("/camera/color/image_rect_color", Image, timeout=100)
        self.cvImage = img.image_to_numpy(self.rosImage)
        self.cvImage = img.bgr_to_rgb(self.cvImage)
        return self.cvImage


    def getROSDepthImage(self):
        """Wait for depth images
        """
        bridge = CvBridge()
        self.rosDepthImage = rospy.wait_for_message("/camera/depth/image_rect_raw", Image, timeout=100)
        self.cvDepthImage = self.bridge.imgmsg_to_cv2(self.rosDepthImage, "passthrough")
        print(self.cvDepthImage.size())
        return self.cvDepthImage