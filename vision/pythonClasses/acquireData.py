import rospy
from sensor_msgs.msg import Image
from . import img
from . import imageManipulation

class acquireImage():
    """Class to acquire images from ROS
    """
    def __init__(self):
        """Class constructor
        """
        self.rosImage = []
        self.cvImage = []

    def getROSImage(self):
        """Wait for images
        """
        self.rosImage = rospy.wait_for_message("/camera/color/image_rect_color", Image, timeout=100)

    def convertImage(self):
        """Converts image into numpy array
        """
        self.cvImage = img.image_to_numpy(self.rosImage)
        self.cvImage = img.bgr_to_rgb(self.cvImage)
        

    def returnImage(self):
        """Returns images in the shape of numpy arrays

        Returns:
            np.array: Returns the input image for the image node
        """
        self.getROSImage()
        self.convertImage()
        return self.cvImage