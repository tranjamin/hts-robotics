from __future__ import annotations
import os
import time
import argparse
import math
import numpy as np
import open3d as o3d
import time
from scipy.spatial.transform import Rotation

from graspnetAPI.grasp import Grasp
from graspnetAPI import GraspGroup
from gsnet import AnyGrasp


from utils import display_pointcloud

checkpoint_path = "/ros2_ws/src/hts_anygrasp/hts_anygrasp/log/checkpoint_detection.tar"
cloud_path = "/ros2_ws/src/pointclouds/volumetric_flask.npz"

# configs for anygrasp
cfgs = argparse.Namespace(
    checkpoint_path=checkpoint_path,
    max_gripper_width=0.1,
    gripper_height=0.03,
    top_down_grasp=False,
    debug=True, # was true
)

anygrasp = AnyGrasp(cfgs)
anygrasp.load_net()

from_file = np.load(cloud_path)
points = from_file['points']
colours = np.zeros_like(points, dtype=np.float32)

# filter according to z
z_coords = points[:, 2]
y_coords = points[:, 1]
x_coords = points[:, 0]
mask = (z_coords > 0.001) & (z_coords < 100) & ((0.3 - x_coords)**2 + (0.0 - y_coords)**2 < 0.08**2)

cropped_points = points[mask].astype(np.float32)
cropped_colours = colours[mask].astype(np.float32)
uncropped_points = points[~mask].astype(np.float32)
uncropped_colours = colours[~mask].astype(np.float32)

# cropped_points = points
# cropped_colours = colours

display_pointcloud(cropped_points, save=False, description="Cropped Point Cloud")

# set workspace to filter output grasps
xmin, xmax = -2, 2
ymin, ymax = -2, 2
zmin, zmax = 0.05, 50
lims = [xmin, xmax, ymin, ymax, zmin, zmax]

gg, cloud = anygrasp.get_grasp(
    cropped_points, cropped_colours, 
    lims=lims, 
    apply_object_mask=False, 
    dense_grasp=True, 
    collision_detection=True
)

def visualise(grasp):
    gg = GraspGroup()

    gg.add(grasp)

    grippers = gg.to_open3d_geometry_list()

    trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    for ind, g in enumerate(grippers):
        g.transform(trans_mat)

    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.1,      # length of the axes
    )

    cloud.transform(trans_mat)

    o3d.visualization.draw_geometries([*grippers, cloud, origin_frame])

def map_grasp(grasp, flip_z=False, delta_back=0.0, delta_up=0.0):
    grasp_rotation = Rotation.from_matrix(grasp.rotation_matrix)

    offset_rotation = Rotation.from_euler('y', 90, degrees=True)
    flip_rotation = Rotation.from_euler('z', 180, degrees=True)

    final_rotation = grasp_rotation * offset_rotation

    # detect if camera is pointing downwards
    x_axis = final_rotation.as_matrix()[:, 0] # x_axis[2] > 0 ==> camera is facing upwards
    if (x_axis[2] < 0) ^ flip_z: # we are up and want to be down, or vice versa
        final_rotation =  final_rotation * flip_rotation

    # local axes:
        # z points in the direction of grasp attack
        # y is perpendicular to z in the horizontal plane (action of gripper)
        # x points vertical

    offset_translation = np.array([0, 0, -delta_back])
    grasp.translation[2] += delta_up
    final_translation = grasp.translation + final_rotation.as_matrix() @ offset_translation
    final_quaternion = final_rotation.as_quat()

    return final_translation, final_quaternion

# replace cloud with rainbow point clouds
rainbow_cloud = o3d.geometry.PointCloud()
rainbow_cloud.points = o3d.utility.Vector3dVector(cropped_points)

# for g in gg:
#     visualise(g)

visualise(gg[11])
map_grasp(gg[11])
input()