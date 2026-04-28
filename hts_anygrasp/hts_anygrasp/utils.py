from __future__ import annotations
import os
import numpy as np
import open3d as o3d

from ament_index_python.packages import get_package_prefix

pkg_prefix = get_package_prefix("hts_anygrasp")
lib_path = os.path.join(pkg_prefix, "lib", "hts_anygrasp")
os.environ["LD_LIBRARY_PATH"] = (lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", ""))
checkpoint_path = os.path.join(pkg_prefix, "share/hts_anygrasp/checkpoint_detection.tar")

from graspnetAPI import GraspGroup

def display_grasps(gg: GraspGroup, cloud: o3d.cuda.pybind.geometry.PointCloud, 
                   only_first: bool=False, origin_position: list[float]=[0,0,0], description: str=""
                   ):
    try:
        trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        cloud.transform(trans_mat)
        grippers = gg.to_open3d_geometry_list()
        for gripper in grippers:
            gripper.transform(trans_mat)
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1,      # length of the axes
            origin=origin_position
        )
        if not only_first:
            o3d.visualization.draw_geometries([*grippers, cloud, origin_frame], window_name=description)
        else:
            o3d.visualization.draw_geometries([grippers[0], cloud, origin_frame], window_name=description)
    except Exception as e:
        pass

def display_pointcloud(points, colors=None, save=False, filename=None, origin_position=[0,0,0], description=""):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors) 
    if save:
        o3d.io.write_point_cloud(f"{filename}.pcd", pcd, write_ascii=True)
        np.savez(f"{filename}.npz", points=points, colors=colors)
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=origin_position)
    o3d.visualization.draw_geometries([pcd, origin_frame], window_name=description)

def fast_norgb_pc2_to_numpy(msg):
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('pad', np.float32),
    ])

    cloud = np.frombuffer(msg.data, dtype=dtype)
    points = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=-1)
    mask = np.isfinite(points).all(axis=1)
    points = points[mask]

    return points.astype(np.float32), np.zeros_like(points, dtype=np.float32)

def fast_pc2_to_numpy(msg):
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('pad', np.float32),
        ('rgb', np.float32),
        ('pad2', np.float32),
    ])

    cloud = np.frombuffer(msg.data, dtype=dtype)

    points = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=-1)

    mask = np.isfinite(points).all(axis=1)
    points = points[mask]

    rgb_uint = cloud['rgb'].view(np.uint32)

    r = ((rgb_uint >> 16) & 255).astype(np.float32) / 255.0
    g = ((rgb_uint >> 8) & 255).astype(np.float32) / 255.0
    b = (rgb_uint & 255).astype(np.float32) / 255.0

    colors = np.stack([r, g, b], axis=-1)[mask]

    return points.astype(np.float32), colors.astype(np.float32)