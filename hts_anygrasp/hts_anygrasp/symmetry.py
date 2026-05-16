# regular imports
from __future__ import annotations
import numpy as np
import numpy.typing as npt
from typing import Any
import copy
import open3d as o3d
from graspnetAPI import GraspGroup as GraspNetGroup
from graspnetAPI.grasp import Grasp

class SymmetryGroup():
    """A class which generates extra grasps by symmetries"""

    def __init__(self, cloud: o3d.cuda.pybind.geometry.PointCloud, gg: GraspNetGroup):
        """
        Parameters:
            cloud: the cropped point cloud of the object
            gg: the grasp group containing grasps
        """
        self._cloud: o3d.cuda.pybind.geometry.PointCloud = cloud
        self._gg: GraspNetGroup = gg
    
    def get_centre(self) -> npt.NDArray[np.float64]:
        """Retrieves the centroid of the cloud"""
        return self._cloud.get_center()
    
    def grasps(self) -> GraspNetGroup:
        return self._gg
    
    def rotate_about_centre(self, rotation_angle: float) -> SymmetryGroup:
        """
        Rotates the point cloud and grasps about the centre.

        Parameters:
            rotation_angle: the clockwise angle to rotate the point cloud and grasps by, in degrees
        
        Returns:
            a new SymmetryGroup containing a rotated cloud and grasps
        """

        rotation_matrix_o3d: Any = self._cloud.get_rotation_matrix_from_xyz((0, 0, rotation_angle * np.pi / 180))
        centre: npt.NDArray[np.float64] = self.get_centre()

        # first translate so the centroid is at the origin
        initial_translation: npt.NDArray[np.float64] = np.block([
            [np.eye(3), -centre.reshape(3,1)],
            [np.zeros((1,3)), 1]
        ])

        # rotatation transformation
        homog_rotation: npt.NDArray[np.float64] = np.block([
            [rotation_matrix_o3d, np.zeros((3,1))],
            [np.zeros((1,3)), 1]
        ]) # type: ignore

        # then translate back to the centroid position
        final_translation: npt.NDArray[np.float64] = np.block([
            [np.eye(3), centre.reshape(3,1)],
            [np.zeros((1,3)), 1]
        ])

        # compose the total transformation
        transformation_matrix: npt.NDArray[np.float64] = final_translation @ (homog_rotation @ initial_translation)

        # transform cloud
        transformed_cloud: o3d.cuda.pybind.geometry.PointCloud = copy.deepcopy(self._cloud)
        transformed_cloud.rotate(rotation_matrix_o3d)

        # transform grasp group
        transformed_gg: GraspNetGroup = GraspNetGroup()
        for grasp in self.grasps():
            transformed_grasp: Grasp = copy.deepcopy(grasp)
            transformed_grasp.transform(transformation_matrix)
            transformed_gg.add(transformed_grasp)
        
        return SymmetryGroup(transformed_cloud, transformed_gg)
    
    def clouds_are_similar(self, group2: SymmetryGroup, thresh: float) -> bool:
        """
        Determines if two point clouds are sufficiently similar

        Parameters:
            group2: the second symmetry group to compare
            thresh: the maximum average chamfer distance to consider as two clouds being similar
        Returns:
            whether the point clouds are similar or not
        """
        chamfer_distance: Any = self._cloud.compute_point_cloud_distance(group2._cloud)
        mean_distance: float = np.array(chamfer_distance).mean()
        
        return mean_distance < thresh
    
    