import open3d as o3d
import numpy as np

# Read PCD.
pcd = o3d.io.read_point_cloud("/Users/robotics_qi/Data/kitti_dataset/lidar/pcds/000888.pcd")
# pcd = o3d.io.read_point_cloud("/Users/robotics_qi/Downloads/fragment.ply")
# print(pcd)

# Help know Open3D
# help(o3d)

# Test the IO Function: Write Open3D PCD

# o3d.io.write_point_cloud("KITTI_TEST.pcd", pcd)
# o3d.visualization.draw_geometries([pcd], zoom=0.3412,
#                                  front=[0.4257, -0.2125, -0.8795],
#                                  lookat=[2.6172, 2.0475, 1.532],
#                                  up=[-0.0694, -0.9768, 0.2024])

#o3d.visualization.draw_geometries()