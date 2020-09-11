import open3d as o3d
#from open3d import *
import numpy as np
#help(o3d)

pcd = o3d.io.read_point_cloud("/home/robotics/Data/kitti_dataset/odometry/lidar/pcd_format/pcds/000888.pcd")

#draw_geometries([pcd])

# Nevers
o3d.visualization.draw_geometries([pcd])
