import numpy as np
import open3d as o3d

from .acquireData import acquireImage

    
class pointCloudProcessing():

    def __init__(self):
        a = 0
    
    
    def pointCloudGenerate(self, rgbImage):
        ai = acquireImage()

        depthImage = ai.getROSDepthImage()

        fx = 686.602445530949
        fy = 686.602445530949
        cx = 638.477030032085
        cy = 359.464552799678
        w = 640
        h = 480
        intrinsicsObj = o3d.camera.PinholeCameraIntrinsic()

        intrinsicsObj.set_intrinsics(w, h, fx, fy, cx, cy)

        imgo3d = o3d.geometry.Image(rgbImage.astype(np.uint8))
        depth3d = o3d.geometry.Image(depthImage.astype(np.float32))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(imgo3d, depth3d, convert_rgb_to_intensity=False)
        cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsicsObj)

        return cloud
        
    def downscaleCloud(self, pointCloud):
        octreeData = o3d.cuda.pybind.geometry.Octree()
        print(pointCloud)
        o3d.visualization.draw_geometries([pointCloud])

        o3d.geometry.Octree.convert_from_point_cloud(octreeData, pointCloud)
        print(octreeData)
        return octreeData
