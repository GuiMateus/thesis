import numpy as np
import open3d as o3d
import cv2
import matplotlib.pyplot as plt
from .imageManipulation import imageManipulation


    
class pointCloudProcessing():

    def __init__(self):
        a = 0
    
    
    def pointCloudGenerate(self, rgbImage, depthImage, intensity=False):

        print(len(rgbImage[0]))
        if len(rgbImage) == len(depthImage) and len(rgbImage[0]) == len(depthImage[0]):        
            fx = 725.418477077181
            fy = 697.956763056449
            cx = 316.369990062088
            cy = 250.294460891937
            w = 640
            h = 480
            intrinsicsObj = o3d.camera.PinholeCameraIntrinsic()

            intrinsicsObj.set_intrinsics(w, h, fx, fy, cx, cy)

            cv2.cvtColor(rgbImage, cv2.COLOR_BGR2RGB)
            imgo3d = o3d.geometry.Image(rgbImage.astype(np.uint8))
            depth3d = o3d.geometry.Image(depthImage.astype(np.float32))



            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(imgo3d, depth3d, convert_rgb_to_intensity=intensity)
            cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsicsObj)

            cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            return cloud
    
        else:
            return None
        
    def downscaleCloud(self, pointCloud):
        octreeData = o3d.cuda.pybind.geometry.Octree(max_depth=8)
        print(pointCloud)

        o3d.geometry.Octree.convert_from_point_cloud(octreeData, pointCloud, size_expand=0.0001)
        
        # voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pointCloud,
        #                                                       voxel_size=0.01)

        # pcd_tree = o3d.geometry.KDTreeFlann(pointCloud)

        o3d.visualization.draw_geometries([pointCloud])
        o3d.io.write_point_cloud("/home/gui/octree.ply", pointCloud)

        print(octreeData)
        return octreeData

    def filterMasks(self, mask, depthImage):
        pointCloud = self.pointCloudGenerate(mask, depthImage, intensity=True)

        if pointCloud is not None:
            colors = np.asarray(pointCloud.colors)
            print(np.where(colors[:,0] > 0))
            pointSelection = pointCloud.select_by_index(np.where(colors[:,0] > float(0))[0])

            if not pointSelection.is_empty():

                im = imageManipulation()

                print(pointSelection)
                # o3d.visualization.draw_geometries([pointSelection])

                with o3d.utility.VerbosityContextManager(
                    o3d.utility.VerbosityLevel.Debug) as cm:
                    labels = np.array(
                        pointSelection.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

                max_label = labels.max()
                print(f"point cloud has {max_label + 1} clusters")
                colorsCluster = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
                colorsCluster[labels < 0] = 0
                pointSelection.colors = o3d.utility.Vector3dVector(colorsCluster[:, :3])
                colorsSelection = np.asarray(pointSelection.colors)

                o3d.visualization.draw_geometries([pointSelection])

                npCloud = np.asarray(pointSelection.points)
                npColours = np.asarray(pointSelection.colors)
                print("this is the cloud" + str(npColours))

