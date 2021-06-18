import numpy as np
import open3d as o3d
import cv2
import os
import time
import matplotlib.pyplot as plt
from .imageManipulation import imageManipulation


    
class pointCloudProcessing():

    def __init__(self):
        self.intrinsicMatrix = np.matrix([[725.418477077181, 0, 316.369990062088], 
        [0, 697.956763056449, 250.294460891937],
        [0, 0, 1]])
        self.cloud = []
    
    
    def pointCloudGenerate(self, rgbImage, depthImage, intensity=False):

        print(len(rgbImage[0]))
        if len(rgbImage) == len(depthImage) and len(rgbImage[0]) == len(depthImage[0]):        
            fx = self.intrinsicMatrix[0,0]
            fy = self.intrinsicMatrix[1,1]
            cx = self.intrinsicMatrix[0,2]
            cy = self.intrinsicMatrix[1,2]
            w = 1280
            h = 720
            intrinsicsObj = o3d.camera.PinholeCameraIntrinsic()

            intrinsicsObj.set_intrinsics(w, h, fx, fy, cx, cy)

            rgbImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2RGB)
            imgo3d = o3d.geometry.Image(rgbImage.astype(np.uint8))
            depth3d = o3d.geometry.Image(depthImage.astype(np.float32))



            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(imgo3d, depth3d, convert_rgb_to_intensity=intensity)
            cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsicsObj)

            cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -0.75, 0], [0, 0, 0, 1]])
            self.cloud = cloud
    
        else:
            return None
    
    def saveCloud(self):
        """Create pointcloud and transform it into octree
        """
        octreeData = o3d.cuda.pybind.geometry.Octree(max_depth=8)
        o3d.geometry.Octree.convert_from_point_cloud(octreeData, self.cloud, size_expand=0.0001)
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="CloudSaverWindow", width=1280, height=720, visible=False)
        vis.add_geometry(octreeData)
        vis.update_geometry(octreeData)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(".environmentReconstruction/cloud.png", True)


        
    def downscaleCloud(self, pointCloud):
        octreeData = o3d.cuda.pybind.geometry.Octree(max_depth=16)
        print(pointCloud)

        o3d.geometry.Octree.convert_from_point_cloud(octreeData, pointCloud, size_expand=0.0001)
        
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pointCloud,
                                                              voxel_size=0.005)

        o3d.visualization.draw_geometries([pointCloud])


        print(octreeData)
        return octreeData
