from __future__ import annotations
import os
import numpy as np
import numpy.typing as npt
from typing import Any

from sympy import Point

try:
    import open3d as o3d
    from ament_index_python.packages import get_package_prefix
    from sensor_msgs.msg import PointCloud2
    from hts_msgs.action import ComputeGraspValidity, RequestGrasp

    pkg_prefix = get_package_prefix("hts_anygrasp")
    lib_path = os.path.join(pkg_prefix, "lib", "hts_anygrasp")
    os.environ["LD_LIBRARY_PATH"] = (lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", ""))
    checkpoint_path = os.path.join(pkg_prefix, "share/hts_anygrasp/checkpoint_detection.tar")

    from graspnetAPI import GraspGroup
except ImportError as e:
    print(f"Received Import Error {e}, continuing")

class FakeLogger():
    """A class which wraps print() and behaves like an rclpy logger"""

    def __init__(self) -> None:
        pass

    def info(self, *args, sep=' ', end='\n', file=None, flush=False):
        print(*args, sep=sep, end=end, file=file, flush=flush)

    def warn(self, *args, sep=' ', end='\n', file=None, flush=False):
        print(*args, sep=sep, end=end, file=file, flush=flush)

class ValidityContext():
    """A class containing all relevant information for grasp selection"""

    def __init__(self, 
                 goal_handle: Any, 
                 grasp_group: 'HTSGraspGroup'=None, 
                 folder: str="", 
                 cloud: o3d.cuda.pybind.geometry.PointCloud | None=None, 
                 request: RequestGrasp.Goal | None=None, 
                 plot: bool=False, 
                 visualise: bool=False, 
                 logger: FakeLogger | Any =FakeLogger(), 
                 is_flipped: bool=False, 
                 save_data: bool=False, 
                 client: Any=None
                 ):
        """
        Parameters:
            goal_handle: the goal handle for the grasp planning request
            grasp_group: the HTSGraspGroup containing all valid grasps
            folder: the main directory to save data to
            cloud: the point cloud for the target object, or None if not used (only for visualisation)
            request: the request for grasp planning, or None if not used
            plot: whether to plot the progress of grasp selection each iteration
            visualise: whether to visualise grasp groups with o3d
            logger: the logging object (either rclpy.logger or FakeLogger) for debugging
            is_flipped: whether this context is selecting flipped grasps
            save_data: whether to save data or not
            client: the evaluation client
        """

        self.logger: FakeLogger | Any = logger
        self.goal_handle: Any = goal_handle
        self.hts_grasp_group: 'HTSGraspGroup' = grasp_group
        self.folder: str = folder
        self.cloud: o3d.cuda.pybind.geometry.PointCloud | None = cloud
        self.request: RequestGrasp.Goal | None = request
        self.is_flipped: bool = is_flipped
        self.plot: bool = plot
        self.client: Any = client
        self.save_data: bool = save_data
        self.visualise: bool = visualise

        self.all_points_certain: bool = False # whether all grasps have been evaluated with 100% certainty
        self.response: RequestGrasp.Result | None = None # the response to send back to grasp planning
        self.pending_results: int = 0 # how many evaluations are currently running

def display_grasps(
        gg: GraspGroup, 
        cloud: o3d.cuda.pybind.geometry.PointCloud, 
        only_first: bool=False, 
        origin_position: list[float]=[0,0,0], 
        description: str=""
    ) -> None:
    """
    Visualises a set of grasps using o3d

    Parameters:
        gg: the grasp group to visualise
        cloud: the cloud to display
        only_first: set to true to only display the first grasp in this group
        origin_position: the position to show the origin axis
        description: the title on the visualisation window
    """

    # convert to o3d geometries
    grippers: Any = gg.to_open3d_geometry_list()
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=origin_position)

    # visualise
    if not only_first:
        o3d.visualization.draw_geometries([*grippers, cloud, origin_frame], window_name=description)
    elif len(grippers):
        o3d.visualization.draw_geometries([grippers[0], cloud, origin_frame], window_name=description)

def display_pointcloud(
        points: npt.NDArray[np.float64], 
        colours: npt.NDArray | None=None, 
        save: bool=False, 
        filename: str="", 
        origin_position: list[float]=[0,0,0], 
        description: str="") -> None:
    """
    Display a pointcloud

    Parameters:
        points: the points of the pointcloud
        colours: the colours of the points, or None if there are no colours
        save: True to save the pointcloud to a file
        filename: the file to save the pointcloud to, if save=True
        origin_position: the position to show the origin axis
        description: the title of the visualisation window
    """
    # create new point cloud
    pcd: o3d.cuda.pybind.geometry.PointCloud = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colours is not None:
        pcd.colours = o3d.utility.Vector3dVector(colours)
    
    # save the data to a pcd and npz if needed
    if save:
        o3d.io.write_point_cloud(f"{filename}.pcd", pcd, write_ascii=True)
        if colours is not None:
            np.savez(f"{filename}.npz", points=points, colours=colours)
        else:
            np.savez(f"{filename}.npz", points=points)

    # visualise
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=origin_position)
    o3d.visualization.draw_geometries([pcd, origin_frame], window_name=description)

def norgb_pointcloud2numpy(msg: PointCloud2) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """
    Converts an RGB-free PointCloud2 to a numpy array.

    Parameters:
        msg: the PointCloud2 point cloud received. Data must be in the format (x, y, z, padding) with 4 bytes allocated to each field

    Returns:
        a tuple of the points and colours - all points are assigned a colour of black
    """

    # define the structure of the pointcloud
    dtype: np.dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('_', np.float32),
    ])

    # create point cloud from the message
    cloud: Any = np.frombuffer(msg.data, dtype=dtype)

    # convert it to points
    points: npt.NDArray[np.float32] = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=-1)

    # remove all points that are infinite
    points = points[np.isfinite(points).all(axis=1)]

    return points.astype(np.float32), np.zeros_like(points, dtype=np.float32)

def pointcloud2numpy(msg: PointCloud2) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """
    Converts a PointCloud2 to a numpy array.

    Parameters:
        msg: the PointCloud2 point cloud received. Data must be in the format (x, y, z, padding, rgb, padding) with 4 bytes allocated to each field

    Returns:
        a tuple of the points and colours
    """

    # define the structur of the pointcloud
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('_', np.float32),
        ('rgb', np.float32),
        ('__', np.float32),
    ])

    # create point cloud from the message
    cloud: Any = np.frombuffer(msg.data, dtype=dtype)

    # convert it to points
    points: npt.NDArray[np.float64] = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=-1)

    # remove all points that are infinite
    finite_mask : npt.NDArray[np.bool] = np.isfinite(points).all(axis=1)
    points = points[finite_mask]

    # extract individual channels
    rgb_uint: npt.NDArray[np.uint32] = cloud['rgb'].view(np.uint32)
    r = ((rgb_uint >> 0x10) & 0xFF).astype(np.float32) / 255.0
    g = ((rgb_uint >> 0x08) & 0xFF).astype(np.float32) / 255.0
    b = (rgb_uint & 0xFF).astype(np.float32) / 255.0

    # convert to colours
    colours: npt.NDArray[np.float32] = np.stack([r, g, b], axis=-1)
    colours = colours[finite_mask]

    # return points and colours
    return points.astype(np.float32), colours.astype(np.float32)