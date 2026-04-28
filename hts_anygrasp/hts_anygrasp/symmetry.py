# regular imports
from __future__ import annotations
import numpy as np
import copy
import open3d as o3d

# hts robotics imports
from graspnetAPI import GraspGroup

class SymmetryGroup():
    def __init__(self, cloud: o3d.cuda.pybind.geometry.PointCloud, gg: GraspGroup, logger):
        self._cloud: o3d.cuda.pybind.geometry.PointCloud = cloud
        self._gg = gg
        self._logger = logger
    
    def get_logger(self):
        return self._logger
    
    def get_centre(self) -> np.ndarray:
        return self._cloud.get_center()
    
    def rotate_about_centre(self, rotation_angle: float) -> SymmetryGroup:
        rotation_matrix_o3d = self._cloud.get_rotation_matrix_from_xyz((0, 0, rotation_angle * np.pi / 180))

        centre = self.get_centre()
        initial_translation = np.block([
            [np.eye(3), -centre.reshape(3,1)],
            [np.zeros((1,3)), 1]
        ])
        homog_rotation = np.block([
            [rotation_matrix_o3d, np.zeros((3,1))],
            [np.zeros((1,3)), 1]
        ])
        final_translation = np.block([
            [np.eye(3), centre.reshape(3,1)],
            [np.zeros((1,3)), 1]
        ])

        transformation_matrix = final_translation @ (homog_rotation @ initial_translation)

        transformed_cloud = copy.deepcopy(self._cloud)
        transformed_cloud.rotate(rotation_matrix_o3d)

        transformed_gg = GraspGroup()

        for grasp in self.grasps():
            transformed_grasp = copy.deepcopy(grasp)
            transformed_grasp.transform(transformation_matrix)

            transformed_gg.add(transformed_grasp)
        
        return SymmetryGroup(transformed_cloud, transformed_gg, self._logger)
    
    def clouds_are_similar(self, group2: SymmetryGroup, thresh: float) -> bool:
        chamfer_distance = self._cloud.compute_point_cloud_distance(group2._cloud)
        mean_distance = np.array(chamfer_distance).mean()
        self.get_logger().info(f"Point Cloud Distance is: {mean_distance}")
        
        return mean_distance < thresh
    
    def grasps(self):
        return self._gg
    