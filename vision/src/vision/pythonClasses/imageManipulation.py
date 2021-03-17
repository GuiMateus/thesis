import numpy as np
import cv2
import open3d as o3d
from pypcd import pypcd
import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2, PointField


class imageManipulation():
    """Class containing utilities for image processing
    """

    def __init__(self):
        self.type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
    
    def toPercent(self, coordinate, dimensionSize):
        """Converts a pixel coordinate to a percentage 
        corresponding to an image size

        Args:
            coordinate (float): Value in pixel space
            dimensionSize (int): Size of a dimension of the image (width or height)

        Returns:
            (float): Percentage value corresponding to the pixel position in an image
        """
        percentage = coordinate/dimensionSize
        return percentage

    def toPixel(self, percentage, dimensionSize):
        """Converts an image percentage to a pixel coordinate 

        Args:
            percentage (float): Percentage value
            dimensionSize (int): Size of a dimension of the image (width or height)

        Returns:
            (float): Pixel value corresponding to the percentage
        """
        pixel = percentage * dimensionSize
        return pixel

    def cropImage(self, inputImage, x, y, width, height):
        """Crops an image

        Args:
            inputImage (np.array): [Original image]
            x (float): Upper left x of cropping region
            y (float): Upper left y of cropping region
            width (float): Width of cropping region
            height (float): Height of cropping region

        Returns:
            (np.array): Cropped segment of the image
        """


        width = int(width)
        height = int(height)
        sumY = y+width
        sumX = x+height

        if x < 0:
            width = width + x
            x = 0
        if y < 0:
            height = y + height
            y = 0
        if sumY > inputImage.shape[0]:
            sumY = inputImage.shape[0]
        if sumX > inputImage.shape[1]:
            sumX = inputImage.shape[1]

        croppedImage = inputImage[y:y+height, x:x+width]
        return croppedImage

    def extractBbox(self, bbox):
        """Extracts x, y, w, h coordinates from a bbox method

        Args:
            bbox (list): Contains x, y, w, h

        Returns:
            (float): Values of x, y, w, h
        """
        x, y, w, h = bbox
        return x, y, w, h

    def compressBbox(self, x, y, w, h):
        """Compresses x, y, w, h of a bounding box into a bbox method, making value parsing easier

        Args:
            x (float): x coordinate of the center of a bounding box
            y (float): y coordinate of the center of a bounding box
            w (float): width of a bounding box
            h (float): height of a bounding box

        Returns:
            (list): Contains x, y, w, h
        """
        bbox = (x, y, w, h)
        return bbox

    def getImageShape(self, image):
        x = image.shape[0]
        y = image.shape[1]

        return x, y

    def generatePointCloud(self, input_rgb, input_depth):
        cv2.imshow("rgb", input_rgb)
        cv2.imshow("depth", input_depth)
        cv2.waitKey(1)
        fx = 686.602445530949
        fy = 686.602445530949
        cx = 638.477030032085
        cy = 359.464552799678
        intrinsicsObj = o3d.camera.PinholeCameraIntrinsic()

        intrinsicsObj.set_intrinsics(640, 480, fx, fy, cx, cy)

        imgo3d = o3d.geometry.Image(input_rgb.astype(np.uint8))
        depth3d = o3d.geometry.Image(input_depth.astype(np.float32))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            imgo3d, depth3d, convert_rgb_to_intensity=False)
        cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intrinsicsObj)

        # flip the orientation, so it looks upright, not upside-down
        cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        colors = np.asarray(cloud.colors) * 255

        rgb = colors.astype(np.uint32)
        encoded_colors = np.array((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | (rgb[:, 2] << 0),
                    dtype=np.uint32)
        encoded_colors = np.cast[np.float32](encoded_colors)

        points = np.asarray(cloud.points)

        new_data = np.hstack((points[:, 0, np.newaxis].astype(np.float32), points[:, 1, np.newaxis].astype(
            np.float32), points[:, 2, np.newaxis].astype(np.float32), encoded_colors[:, np.newaxis]))

        new_cloud = pypcd.make_xyz_rgb_point_cloud(new_data)
        print(new_cloud)

        # print(new_data)
        if new_cloud is not None:
            outgoingCloud = self.array_to_pointcloud2(new_cloud)

            if outgoingCloud is not None:
                
                print("Ever here?")
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "base_camera_link"

                outgoingCloud.header = header
        return outgoingCloud


    def array_to_pointcloud2(self, cloud_arr, stamp=None, frame_id=None, merge_rgb=False):
        '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
        '''

        # make it 2d (even if height will be 1)
        cloud_arr = np.atleast_2d(cloud_arr)

        cloud_msg = PointCloud2()

        if stamp is not None:
            cloud_msg.header.stamp = stamp
        if frame_id is not None:
            cloud_msg.header.frame_id = frame_id
        cloud_msg.height = cloud_arr.shape[0]
        cloud_msg.width = cloud_arr.shape[1]
        cloud_msg.fields = self.arr_to_fields(cloud_arr)
        cloud_msg.is_bigendian = False # assumption
        cloud_msg.point_step = cloud_arr.dtype.itemsize
        cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
        if cloud_arr.dtype.names is not None:
            cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
            cloud_msg.data = cloud_arr.tostring()
            return cloud_msg

    def arr_to_fields(self, cloud_arr):
        '''Convert a numpy record datatype into a list of PointFields.
        '''
        nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in self.type_mappings)
        fields = []
        if cloud_arr.dtype.names is not None:
            for field_name in cloud_arr.dtype.names:
                np_field_type, field_offset = cloud_arr.dtype.fields[field_name]
                pf = PointField()
                pf.name = field_name
                pf.datatype = nptype_to_pftype[np_field_type]
                pf.offset = field_offset
                pf.count = 1 # is this ever more than one?
                fields.append(pf)
        return fields

   

    # def toOriginalImage(self, image, masks)
    #     originalMasks = []
    #     networkWidth, networkHeight = self.deepLabDimensions
        
    #     # Get the size of the original input image
    #     imageHeight = image.shape[0]
    #     imageWidth = image.shape[1]

    #     projectedWidth = masks[1].size()
    #     projectedHeight = masks[0].size()
       

    #     # Find the image ratio 
    #     ratio = imageHeight/imageWidth

    #     # Convert all the detections to original image dimensions
    #     for projectedY in range(0, projectedHeight):
    #         for projectedX in range(0, projectedWidth):
    #             # Remove outliers
    #             if projectedX > 0 and projectedX < 10000 and projectedY > 0 and projectedY < 10000 and projectedWidth > 0 and projectedWidth < 10000 and projectedHeight > 0 and projectedHeight < 10000:

    #                 # Convert bounding boxes to percentages
    #                 percentageX = im.toPercent(projectedX, networkWidth)
    #                 percentageY = im.toPercent(projectedY, networkHeight)
    #                 percentageHeight = im.toPercent(projectedHeight, networkHeight)
    #                 percentageWidth = im.toPercent(projectedWidth, networkWidth)

    #                 # Convert percentages to original image pixel values and fix width/height ratio
    #                 originalX = im.toPixel(percentageX, imageWidth)
    #                 originalY = im.toPixel(percentageY, imageHeight)
    #                 originalWidth = im.toPixel(percentageWidth, imageWidth) * ratio
    #                 originalHeight = im.toPixel(percentageHeight, imageHeight) / ratio

    #                 origina