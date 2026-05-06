from __future__ import annotations
import os
import time
import argparse
import math
import numpy as np
import open3d as o3d
import time

from graspnetAPI.grasp import Grasp as GraspNetGrasp
from graspnetAPI import GraspGroup as GraspNetGroup
from gsnet import AnyGrasp

from utils import display_pointcloud

checkpoint_path = "/ros2_ws/src/hts_anygrasp/hts_anygrasp/log/checkpoint_detection.tar"
cloud_path = "/ros2_ws/src/pointclouds/volumetric_flask.npz"

# configs for anygrasp
cfgs = argparse.Namespace(
    checkpoint_path=checkpoint_path,
    max_gripper_width=0.1,
    gripper_height=0.03,
    top_down_grasp=True,
    debug=True, # was true
)

anygrasp = AnyGrasp(cfgs)
anygrasp.load_net()

from_file = np.load(cloud_path)
points = from_file['points']
colors = np.zeros_like(points, dtype=np.float32)

# filter according to z
z_coords = points[:, 2]
y_coords = points[:, 1]
x_coords = points[:, 0]
mask = (z_coords > 0.001) & (z_coords < 100) & ((0.3 - x_coords)**2 + (0.0 - y_coords)**2 < 0.08**2)

cropped_points = points[mask].astype(np.float32)
cropped_colors = colors[mask].astype(np.float32)
uncropped_points = points[~mask].astype(np.float32)
uncropped_colors = colors[~mask].astype(np.float32)

# cropped_points = points
# cropped_colors = colors

display_pointcloud(cropped_points, save=False, description="Cropped Point Cloud")

# set workspace to filter output grasps
xmin, xmax = -2, 2
ymin, ymax = -2, 2
zmin, zmax = 0.05, 50
lims = [xmin, xmax, ymin, ymax, zmin, zmax]

gg, cloud = anygrasp.get_grasp(
    cropped_points, cropped_colors, 
    lims=lims, 
    apply_object_mask=False, 
    dense_grasp=True, 
    collision_detection=True
)

# replace cloud with rainbow point clouds
rainbow_cloud = o3d.geometry.PointCloud()
rainbow_cloud.points = o3d.utility.Vector3dVector(cropped_points)
