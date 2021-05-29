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
        # o3d.visualization.draw_geometries([self.cloud])
        octreeData = o3d.cuda.pybind.geometry.Octree(max_depth=4)
        o3d.geometry.Octree.convert_from_point_cloud(octreeData, self.cloud, size_expand=0.0001)


        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="CloudSaverWindow", width=1280, height=720, visible=False)
        vis.add_geometry(octreeData)
        vis.update_geometry(octreeData)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(".environmentReconstruction/cloud.png", True)
        # time.sleep(5)
        # o3d.io.write_point_cloud(".environmentReconstruction/temp.ply", self.cloud)


        
    def downscaleCloud(self, pointCloud):
        octreeData = o3d.cuda.pybind.geometry.Octree(max_depth=16)
        print(pointCloud)

        o3d.geometry.Octree.convert_from_point_cloud(octreeData, pointCloud, size_expand=0.0001)
        
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pointCloud,
                                                              voxel_size=0.005)

        # pcd_tree = o3d.geometry.KDTreeFlann(pointCloud)

        o3d.visualization.draw_geometries([pointCloud])


        # o3d.io.write_point_cloud("/home/gui/octree.ply", octreeData)
        # o3d.io.write_triangle_mesh("/home/gui/octree.ply", octreeData)

        print(octreeData)
        return octreeData

    # def filterMasks(self, mask, depthImage):
    #     pointCloud = self.pointCloudGenerate(mask, depthImage, intensity=True)

    #     if pointCloud is not None:
    #         colors = np.asarray(pointCloud.colors)
    #         print(np.where(colors[:,0] > 0))
    #         pointSelection = pointCloud.select_by_index(np.where(colors[:,0] > float(0))[0])

    #         if not pointSelection.is_empty():

    #             im = imageManipulation()

    #             print(pointSelection)
    #             # o3d.visualization.draw_geometries([pointSelection])

    #             with o3d.utility.VerbosityContextManager(
    #                 o3d.utility.VerbosityLevel.Debug) as cm:
    #                 labels = np.array(
    #                     pointSelection.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    #             max_label = labels.max()
    #             print("point cloud has {max_label + 1} clusters")
    #             colorsCluster = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    #             colorsCluster[labels < 0] = 0
    #             pointSelection.colors = o3d.utility.Vector3dVector(colorsCluster[:, :3])
    #             colorsSelection = np.asarray(pointSelection.colors)

    #             o3d.visualization.draw_geometries([pointSelection])

    #             npCloud = np.asarray(pointSelection.points)
    #             npColours = np.asarray(pointSelection.colors)

    #             x = []
    #             y = []
    #             z = []
    #             i = 0
    #             a = np.array(1)
    #             output = []
                
    #             for point in npCloud:
    #                 point = np.concatenate(point, a)
    #                 output = point * self.intrinsicMatrix
    #                 print(output)
                    
                    

    #             print(npCloud[0])

    #             img = self.points_to_image_torch(npCloud[0], npCloud[1], npColours, sensor_size=(len(mask), len(mask[0])))
    #             # colour = self.colours_to_image_torch(npColours[0], npColours[1], npColours[2], sensor_size=(len(mask), len(mask[0])))



    #             print("this is the cloud" + str(npColours))

    # def points_to_image_torch(self, xs, ys, vs, sensor_size=(640, 480)):
    #     import torch
    #     xt, yt, colours = torch.from_numpy(xs), torch.from_numpy(ys), torch.from_numpy(vs)
    #     xt, yt = xt.long(), yt.long()
    #     img = torch.zeros(sensor_size)
    #     img.index_put_((yt, xt), colours, accumulate=True)
    #     img = img.numpy()
    #     cv2.imshow("aa", img)
    #     cv2.waitKey(4000)
    #     return img

    # def colours_to_image_torch(self, r, g, b, sensor_size=(640, 480)):
    #     import torch
    #     rt, gt, bt = torch.from_numpy(r), torch.from_numpy(g), torch.from_numpy(b)
    #     colour = torch.zeros(sensor_size)
    #     colour.index_put_((yt, xt), accumulate=True)
    #     colour = colour.numpy()
    #     return colour  
