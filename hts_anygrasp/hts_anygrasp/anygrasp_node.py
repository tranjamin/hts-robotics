from __future__ import annotations
import os
import time
import argparse
import math
from typing import Any
import numpy as np
import numpy.typing as npt
import open3d as o3d
import time
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("agg")

from ament_index_python.packages import get_package_prefix
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from hts_msgs.srv import DisplayCloud
from hts_msgs.action import ComputeGraspValidity, RequestGrasp
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose

from graspnetAPI.grasp import Grasp as GraspNetGrasp
from graspnetAPI import GraspGroup as GraspNetGroup

pkg_prefix = get_package_prefix("hts_anygrasp")
checkpoint_path = os.path.join(pkg_prefix, "share/hts_anygrasp/checkpoint_detection.tar")
lib_path = os.path.join(pkg_prefix, "lib", "hts_anygrasp")
os.environ["LD_LIBRARY_PATH"] = (lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", ""))

LOAD_ANYGRASP=False
if LOAD_ANYGRASP:
    from gsnet import AnyGrasp
else:
    print("Not loading anygrasp")

from .hts_grasps import HTSGrasp, HTSGraspGroup
from .symmetry import SymmetryGroup

from .utils import display_grasps, display_pointcloud, norgb_pointcloud2numpy, pointcloud2numpy, ValidityContext
from .grasp_selection import DualGPGraspSelector, GPGraspSelector, EpsilonGreedyUCB, LognormalPlanningTimeModel, SequentialGraspSelector, SequentialAcquisition

class AnyGraspNode(Node):
    def __init__(self):
        super().__init__('hts_anygrasp')
        self.load_parameters()

        self.GLOBAL_ITERATOR: int = 0
        
        # pointcloud to listen on
        self.depth_pointcloud_: PointCloud2 = None

        # load in point cloud from file if necessary
        if self.POINTCLOUD_FROM_FILE:
            from_file = np.load(self.POINTCLOUD_FILE)
            self.file_points = from_file['points']
            if not self.NO_RGB:
                self.file_colours = from_file['colours']

        # configs for anygrasp
        cfgs = argparse.Namespace(
            checkpoint_path=checkpoint_path,
            max_gripper_width=max(0, min(0.1, self.MAX_GRIPPER_WIDTH)),
            gripper_height=self.GRIPPER_HEIGHT,
            top_down_grasp=self.TOP_DOWN_GRASP,
            debug=True, # was true
        )

        if not self.OVERRIDE_ANYGRASP:
            self.anygrasp = AnyGrasp(cfgs)
            self.anygrasp.load_net()
            self.anygrasp_lims = [self.X_GRASP_MIN, self.X_GRASP_MAX, self.Y_GRASP_MIN, self.Y_GRASP_MAX, self.Z_GRASP_MIN, self.Z_GRASP_MAX]

        # subscribe to pointcloud
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pointcloud_listener_ = self.create_subscription(PointCloud2, self.POINTCLOUD_TOPIC, self.pointcloud_callback_, qos)

        # ros interfaces
        self.grasp_service_ = ActionServer(self, RequestGrasp, 'request_grasp', self.grasp_callback_)
        self.display_service_ = self.create_service(DisplayCloud, 'display_cloud', self.display_callback_)
        self.grasp_validity_client_ = ActionClient(self, ComputeGraspValidity, "compute_grasp_validity")

        self.get_logger().info("Started AnyGrasp Node")

    def load_parameters(self) -> None:
        """Declares and loads parameters"""

        self.declare_parameter('z_coords_min', 0.001)
        self.declare_parameter('z_coords_max', 100.0)
        self.declare_parameter('x_grasp_min', -1.0)
        self.declare_parameter('x_grasp_max', 1.0)
        self.declare_parameter('y_grasp_min', -1.0)
        self.declare_parameter('y_grasp_max', 1.0)
        self.declare_parameter('z_grasp_min', 0.03)
        self.declare_parameter('z_grasp_max', 1.0)
        self.declare_parameter('apply_object_mask', True)
        self.declare_parameter('apply_collisions', True)
        self.declare_parameter('dense_grasp', True)
        self.declare_parameter('min_grasp_width', 0.01)
        self.declare_parameter('max_grasp_pitch_roll_deg', 5.0)
        self.declare_parameter('nms_translation_thresh', 0.005)
        self.declare_parameter('nms_angle_thresh_deg', 5.0)
        self.declare_parameter('pointcloud_from_file', False)
        self.declare_parameter('pointcloud_file', '')
        self.declare_parameter('mask_radius', 0.05)
        self.declare_parameter('no_rgb', True)
        self.declare_parameter('pointcloud_topic', '/octomap_point_cloud_centers')
        self.declare_parameter('visualise', True)
        self.declare_parameter('max_gripper_width', 0.0)
        self.declare_parameter('gripper_height', 0.03)
        self.declare_parameter('top_down_grasp', False)
        self.declare_parameter('grasp_z_offset', 0.00)
        self.declare_parameter('grasp_axis_offset', 0.14)
        self.declare_parameter('save_data', False)
        self.declare_parameter('symmetry_enable', True)
        self.declare_parameter('symmetry_layer_height', 0.03)
        self.declare_parameter('symmetry_rotation_step', 45)
        self.declare_parameter('symmetry_similarity_threshold', 0.01)
        self.declare_parameter('enable_grasp_selection', True)
        self.declare_parameter('plot_selection_graphs', True)
        self.declare_parameter('planning_time_multiplier', 2.0)
        self.declare_parameter('baseline_planning_time', 0.03)
        self.declare_parameter('baseline_planning_time_move', 0.3)
        self.declare_parameter('acquisition_kappa', 3.0)
        self.declare_parameter('acquisition_eps', 0.1)
        self.declare_parameter('kernel_length_scale_z', 0.03)
        self.declare_parameter('kernel_length_scale_th', 0.8)
        self.declare_parameter('kernel_matern_nu', 2.5)
        self.declare_parameter('total_planning_time_sec', 300)
        self.declare_parameter('acquisition_enable_decay', False)
        self.declare_parameter('acquisition_eps_final', 0.1)
        self.declare_parameter('acquisition_eps_decay_rate', 0.99)
        self.declare_parameter('stability_score_correction_enable', True)
        self.declare_parameter('override_anygrasp', True)
        self.declare_parameter('override_anygrasp_folder', '')

        # config options for point/grasp bounding
        self.Z_COORDS_MIN: float = self.get_parameter('z_coords_min').value
        self.Z_COORDS_MAX: float = self.get_parameter('z_coords_max').value
        self.X_GRASP_MIN: float = self.get_parameter('x_grasp_min').value 
        self.X_GRASP_MAX: float = self.get_parameter('x_grasp_max').value
        self.Y_GRASP_MIN: float = self.get_parameter('y_grasp_min').value 
        self.Y_GRASP_MAX: float = self.get_parameter('y_grasp_max').value
        self.Z_GRASP_MIN: float = self.get_parameter('z_grasp_min').value 
        self.Z_GRASP_MAX: float = self.get_parameter('z_grasp_max').value
        self.MASK_RADIUS: float = self.get_parameter('mask_radius').value

        # config options for anygrasp
        self.APPLY_OBJECT_MASK: bool = self.get_parameter('apply_object_mask').value
        self.APPLY_COLLISIONS: bool = self.get_parameter('apply_collisions').value
        self.DENSE_GRASP: bool = self.get_parameter('dense_grasp').value
        self.TOP_DOWN_GRASP: bool = self.get_parameter('top_down_grasp').value

        # geometric options
        self.MIN_GRASP_WIDTH: float = self.get_parameter('min_grasp_width').value
        self.MAX_GRASP_PITCH_ROLL_DEG: float = self.get_parameter('max_grasp_pitch_roll_deg').value
        self.MAX_GRIPPER_WIDTH: float = self.get_parameter('max_gripper_width').value
        self.GRIPPER_HEIGHT: float = self.get_parameter('gripper_height').value

        # config options for NMS
        self.NMS_TRANSLATION_THRESH: float = self.get_parameter('nms_translation_thresh').value
        self.NMS_ANGLE_THRESH_DEG: float = self.get_parameter('nms_angle_thresh_deg').value

        # config options for point cloud
        self.POINTCLOUD_FROM_FILE: bool = self.get_parameter('pointcloud_from_file').value
        self.POINTCLOUD_FILE: str = self.get_parameter('pointcloud_file').value
        self.NO_RGB: bool = self.get_parameter('no_rgb').value
        self.POINTCLOUD_TOPIC: str = self.get_parameter('pointcloud_topic').value

        # config options for logging
        self.VISUALISE: bool = self.get_parameter('visualise').value
        self.SAVE_DATA: bool = self.get_parameter('save_data').value
        self.PLOT_SELECTION_GRAPHS: bool = self.get_parameter('plot_selection_graphs').value

        # config options for offsetting
        self.GRASP_Z_OFFSET: float = self.get_parameter('grasp_z_offset').value
        self.GRASP_AXIS_OFFSET: float = self.get_parameter('grasp_axis_offset').value

        # config options for symmetry generation
        self.SYMMETRY_ENABLE: bool = self.get_parameter('symmetry_enable').value
        self.SYMMETRY_LAYER_HEIGHT: float = self.get_parameter('symmetry_layer_height').value
        self.SYMMETRY_ROTATION_STEP: int = self.get_parameter('symmetry_rotation_step').value
        self.SYMMETRY_SIMILARITY_THRESHOLD: float = self.get_parameter('symmetry_similarity_threshold').value

        self.STABILITY_SCORE_CORRECTION_ENABLE: bool = self.get_parameter('stability_score_correction_enable').value
        self.OVERRIDE_ANYGRASP: bool = self.get_parameter('override_anygrasp').value
        self.OVERRIDE_ANYGRASP_FOLDER: str = self.get_parameter('override_anygrasp_folder').value

        # config for grasp selection
        self.ENABLE_GRASP_SELECTION: bool = self.get_parameter('enable_grasp_selection').value      
        self.PLOT_SELECTION_GRAPHS: bool = self.get_parameter('plot_selection_graphs').value
        self.PLANNING_TIME_MULTIPLIER: float = self.get_parameter('planning_time_multiplier').value
        self.BASELINE_PLANNING_TIME: float = self.get_parameter('baseline_planning_time').value
        self.BASELINE_PLANNING_TIME_MOVE: float = self.get_parameter('baseline_planning_time_move').value
        self.ACQUISITION_KAPPA: float = self.get_parameter('acquisition_kappa').value
        self.ACQUISITION_EPS: float = self.get_parameter('acquisition_eps').value
        self.KERNEL_LENGTH_SCALE_Z: float = self.get_parameter('kernel_length_scale_z').value
        self.KERNEL_LENGTH_SCALE_TH: float = self.get_parameter('kernel_length_scale_th').value
        self.KERNEL_MATERN_NU: float = self.get_parameter('kernel_matern_nu').value
        self.TOTAL_PLANNING_TIME_SEC: float = self.get_parameter('total_planning_time_sec').value
        self.ACQUISITION_ENABLE_DECAY: bool = self.get_parameter('acquisition_enable_decay').value
        self.ACQUISITION_EPS_FINAL: float = self.get_parameter('acquisition_eps_final').value
        self.ACQUISITION_EPS_DECAY_RATE: float = self.get_parameter('acquisition_eps_decay_rate').value

    def pointcloud_callback_(self, msg: PointCloud2) -> None:
        """
        Stores a pointcloud

        Parameters:
            msg: the point cloud
        """
        self.depth_pointcloud_ = msg

    def display_callback_(self, request: DisplayCloud.Request, response: DisplayCloud.Response) -> DisplayCloud.Response:
        """Callback for the display_cloud service"""

        if self.depth_pointcloud_ is None:
            self.get_logger().info("Found no points")

        points: npt.NDArray[np.float64]
        colours: npt.NDArray[np.float64]

        if self.NO_RGB:
            points, colours = norgb_pointcloud2numpy(self.depth_pointcloud_)
        else:
            points, colours = pointcloud2numpy(self.depth_pointcloud_)

        if points.shape[0] == 0:
            self.get_logger().info("No points found")
            return response

        # Open3D visualization
        display_pointcloud(points, colours)

        if self.SAVE_DATA:
            np.savez(f"src/pointclouds/displayed_cloud_{time.time()}.npz", points=points, colours=colours)

        return response

    def retrieve_pointcloud(self, save_folder: str) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Retrieves the current point cloud, and saves it under full_cloud.npz

        Parameters:
            save_folder: the main directory to save data to
        """
        points: npt.NDArray[np.float64]
        colours: npt.NDArray[np.float64]
        if self.NO_RGB:
            if not self.POINTCLOUD_FROM_FILE:
                points, colours = self.norgb_pointcloud2numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colours = np.zeros_like(points, dtype=np.float64)
            if self.VISUALISE:
                display_pointcloud(points, save=self.SAVE_DATA, filename=f"{save_folder}/full_cloud", description="Full Point Cloud")
        else:
            if not self.POINTCLOUD_FROM_FILE:
                points, colours = self.pointcloud2numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colours = self.file_colours
            if self.VISUALISE:
                display_pointcloud(points, colours, save=self.SAVE_DATA, filename=f"{save_folder}/full_cloud", description="Full Point Cloud")

        return points, colours

    def crop_point_cloud(self, 
        points: npt.NDArray[np.float64], 
        colours: npt.NDArray[np.float64], 
        x: float, y: float, z: float, radius: float, save_folder: str) -> tuple[npt.NDArray[np.float64] | None, npt.NDArray[np.float64] | None]:
        """
        Crops a point cloud as a cylinder, centred at (x,y) with a radius of radius

        Parameters:
            points: the points of the point cloud
            colours: the colours of the point cloud
            x,y,z: the centre of the object
            radius: the radius of the bounding box
            save_folder: the main directory of data being stored

        Return:
            a tuple of the cropped points and colours, or None if the cropped cloud is empty
        """
        # filter according to z
        z_coords: npt.NDArray[np.float64] = points[:, 2]
        y_coords: npt.NDArray[np.float64] = points[:, 1]
        x_coords: npt.NDArray[np.float64] = points[:, 0]
        mask = (z_coords > self.Z_COORDS_MIN) & (z_coords < self.Z_COORDS_MAX) & ((x - x_coords)**2 + (y - y_coords)**2 < radius**2)
        
        cropped_points: npt.NDArray[np.float64] = points[mask].astype(np.float64)
        cropped_colours: npt.NDArray[np.float64] = colours[mask].astype(np.float64)
        uncropped_points: npt.NDArray[np.float64] = points[~mask].astype(np.float64)
        uncropped_colours: npt.NDArray[np.float64] = colours[~mask].astype(np.float64)

        if not cropped_points.shape[0]:
            self.get_logger().error("Cropped pointcloud contains no points")
            return None, None
        
        # show cropped and uncropped pointclouds
        if self.VISUALISE:
            if self.NO_RGB:
                display_pointcloud(cropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                display_pointcloud(uncropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")
            else:
                display_pointcloud(cropped_points, cropped_colours, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                display_pointcloud(uncropped_points, uncropped_colours, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")

        return cropped_points, cropped_colours

    def apply_symmetries(self, gg: GraspNetGroup, cloud: o3d.cuda.pybind.geometry.PointCloud, x: float, y: float, z: float, save_folder: str) -> None:
        """
        Modifies a grasp group to add symmetries.

        Parameters:
            gg: the grasp group to modify
            cloud: the point cloud
            x,y,z: the centre of the object
            save_folder: the main directory of the data being stored
        """

        if self.SYMMETRY_ENABLE: 
            self.get_logger().info("Creating Symmetry Grasps...")
            self.create_symmetry_grasps(gg, cloud)
            self.get_logger().info("Created Symmetry Grasps.")

            unfiltered_gg = gg.nms(
                translation_thresh = self.NMS_TRANSLATION_THRESH,
                rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
            )

            if self.SAVE_DATA:
                with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                    f.write(f"Total num grasps (post-symmetry): {len(gg)}\r\n")
                    f.write(f"Total num grasps after nms (post-symmetry): {len(unfiltered_gg)}\r\n")
                unfiltered_gg.save_npy(f"{save_folder}/grasp_groups/post_symmetry_grasps")
                self.save_grasps_in_polar(gg, save_folder, "post_symmetry")
                self.save_grasps_in_polar(unfiltered_gg, save_folder, "post_symmetry_postnms")
            if self.VISUALISE:
                display_grasps(unfiltered_gg, cloud, origin_position=[x,y,z], description="Post-Symmetry Grasps")

    def filter_grasps(self, gg: GraspNetGroup) -> None:
        """
        Filters grasps according to their angle.

        Parameters:
            gg: the grasp group to filter
        """

        exclude_grasps: list[int] = []
        for ind, grasp in enumerate(gg):
            if grasp.width < self.MIN_GRASP_WIDTH: # exclude grasps by width
                exclude_grasps.append(ind)
                continue

            # exclude grasps by orientation
            roll, pitch, yaw = Rotation.from_matrix(grasp.rotation_matrix).as_euler('xyz', degrees=True)
            if abs(pitch) > self.MAX_GRASP_PITCH_ROLL_DEG and abs(pitch - 180) > self.MAX_GRASP_PITCH_ROLL_DEG:
                exclude_grasps.append(ind)
                continue
            if abs(roll) > self.MAX_GRASP_PITCH_ROLL_DEG and abs(roll - 180) > self.MAX_GRASP_PITCH_ROLL_DEG:
                exclude_grasps.append(ind)
                continue
        gg.remove(exclude_grasps)

    def correct_scores(self, gg: GraspNetGroup, cloud: o3d.cuda.pybind.geometry.PointCloud, save_folder: str) -> None:
        """
        Corrects the scores of the grasp group.

        Parameters:
            gg: the grasp group
            cloud: the point cloud
            save_folder: the main directory of the data being stored
        """

        original_xyz: npt.NDArray[np.float64] = np.array([g.translation for g in gg])
        geometric_centroid = cloud.get_center()

        # gets the original grasp scores
        original_scores: npt.NDArray[np.float64] = np.array(list(gg.scores))
        self.save_grasps_in_polar(gg, save_folder, "Original Grasp Scores", c=gg.scores)

        # calculate a simplified inertia and mass
        SLICE_LAYER_HEIGHT = 0.02
        ASSUMED_THICKNESS = 0.002
        total_mass: float = 0
        total_ixx: float = 0
        layer_base_height: float = self.Z_COORDS_MIN
        layer_masses = []
        layer_rsquareds = []
        while (layer_base_height < self.Z_COORDS_MAX):

            # filter cloud and grasps by height
            bb: Any = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=[-math.inf, -math.inf, layer_base_height],
                max_bound=[math.inf, math.inf, layer_base_height + SLICE_LAYER_HEIGHT]
            )
            layer_cloud: Any = cloud.crop(bb)

            if layer_cloud.is_empty():
                self.get_logger().info(f"Skipping... {layer_cloud}")
                layer_masses.append(0)
                layer_rsquareds.append(0)
                layer_base_height += SLICE_LAYER_HEIGHT
                continue

            # approximate layer as disk
            layer_points: npt.NDArray[np.float64] = np.asarray(layer_cloud.points)
            rsquared: npt.NDArray[np.float64] = np.square(layer_points[:,0] - geometric_centroid[0]) + np.square(layer_points[:,1] - geometric_centroid[1])
            average_rsquared: float= float(np.mean(rsquared))

            # approximate mass
            layer_mass: float = average_rsquared*np.pi*SLICE_LAYER_HEIGHT - (average_rsquared - ASSUMED_THICKNESS)*np.pi*SLICE_LAYER_HEIGHT # alternatively, we could do convex hull
            layer_masses.append(layer_mass)
            layer_rsquareds.append(average_rsquared)
            total_mass += layer_mass

            layer_base_height += SLICE_LAYER_HEIGHT

        # calculate centre of mass from circles
        best_centroid_estimate = 0
        best_centroid_diff = math.inf
        for i in range(len(layer_masses)):
            under_mass = sum(layer_masses[:i])
            over_mass = sum(layer_masses[i:])
            mass_diff = abs(over_mass - under_mass)
            if mass_diff < best_centroid_diff:
                best_centroid_estimate = self.Z_COORDS_MIN + i*SLICE_LAYER_HEIGHT
                best_centroid_diff = mass_diff
        
        self.get_logger().info(f"Estimated Centroid is {best_centroid_estimate}")

        # calculate inertia
        for i, layer_mass in enumerate(layer_masses):
            if layer_mass == 0:
                continue
            layer_ixx: float = 1/4*layer_mass*(layer_rsquareds[i] + (math.sqrt(layer_rsquareds[i]) - ASSUMED_THICKNESS)**2) + 1/3*layer_mass*(SLICE_LAYER_HEIGHT**2)
            shifted_ixx: float = layer_ixx + layer_mass*((best_centroid_estimate - (self.Z_COORDS_MIN + i*SLICE_LAYER_HEIGHT))**2)
            total_ixx += shifted_ixx

        # calculates the distance away from the centroid
        distance_above_centroid: npt.NDArray[np.float64] = original_xyz[:, 2] - best_centroid_estimate

        # calculates the stable score
        self.save_grasps_in_polar(gg, save_folder, "Distance above centroid", c=distance_above_centroid)
        self.save_grasps_in_polar(gg, save_folder, "Inertia", c=total_ixx + total_mass*distance_above_centroid**2)

        # calculates the stable score
        stable_score: npt.NDArray[np.float64] = np.abs(distance_above_centroid)/(np.max(np.abs(distance_above_centroid))*1.5)
        self.save_grasps_in_polar(gg, save_folder, "Estimated Stable Scores", c=stable_score)


        # calculates the grasp score without stable score
        unstable_scores: npt.NDArray[np.float64] = (original_scores/(1 - stable_score)).clip(max=1.0)
        self.save_grasps_in_polar(gg, save_folder, "Pre-Stability Grasp Scores", c=unstable_scores)

        # calculate the estimated (sqrt) lambda
        lambdas: npt.NDArray[np.float64] = -np.sign(distance_above_centroid)*np.sqrt(total_mass*9.81*np.abs(distance_above_centroid)/(total_ixx + total_mass*distance_above_centroid**2))

        # normalise lambdas to [-0.5, 0.5]
        self.get_logger().info(f"Total mass is {total_mass} total ixx is {total_ixx}")
        self.get_logger().info(f"Best lamba is {np.max(np.abs(lambdas))} all are {lambdas}")

        # negative lambdas are stable, with bigger values
        # positive lambdas are unstable, with bigger values
        most_positive = np.max(lambdas)
        most_negative = np.min(lambdas)

        lambdas_list = list(lambdas)

        for i, l in enumerate(lambdas_list):
            if most_negative < 0 and l < 0:
                lambdas_list[i] = l/np.abs(most_negative)/2 # noramlise to between -0.5 and 0
            elif most_positive > 0 and l > 0:
                lambdas_list[i] = l/np.abs(most_positive)/2 # normalise to between 0 and 0.5

        lambdas = np.array(lambdas_list)
        lambdas = 1 - (lambdas + 0.5) # scale to [0, 1], larger values are more stable
        self.get_logger().info(f"All new lambdas are {lambdas}")

        self.save_grasps_in_polar(gg, save_folder, "Lambda Values", c=lambdas)

        # lambdas = lambdas.clip(min=0.0)

        # multiply grasp scores by this
        gg.scores = gg.scores*(lambdas)
        self.save_grasps_in_polar(gg, save_folder, "Grasp Score Lambda Corrected", c=gg.scores)

        dist_from_centre = []
        grasp_factors = []

        # now we determine the grasps whose region does not pass through the centre of the object in 2D
        for grasp in gg:
            # we figure out the direction of closing in 2D
            closing_direction = grasp.rotation_matrix[:2, 1]
            closing_direction /= np.linalg.norm(closing_direction)
            grasp_centre = grasp.translation[:2]

            # get key points:
            left_finger_tip = grasp.translation + np.array([grasp.depth, -grasp.width/2, 0]) @ grasp.rotation_matrix.T
            right_finger_tip = grasp.translation + np.array([grasp.depth, grasp.width/2, 0]) @ grasp.rotation_matrix.T
            tip_centre = grasp.translation + np.array([grasp.depth, 0, 0]) @ grasp.rotation_matrix.T

            # we project it to the perp distance
            dist = grasp_centre - geometric_centroid[:2]
            min_dist = dist - np.dot(dist, closing_direction) * closing_direction
            

            # we need to decide whether our grippers are on the opposite side or not
            # min_dist = (1 if sign(min_dist) == sign(min_finger_dist) else -1)*min_finger_dist

            # clip everything
            grasp_factor = np.linalg.norm(min_dist)
            dist_from_centre.append(grasp_factor)
            grasp_factors.append(grasp_factor)
            # self.get_logger().info(f"Index {len(grasp_factors) - 1} Closing direction is {closing_direction}, grasp centre is {grasp.translation[:2]} centroid is {geometric_centroid[:2]} min dist is {np.linalg.norm(min_dist)}")
            # self.get_logger().info(f"Dist from centre is {grasp_factor}")
        
        grasp_factors = np.array(grasp_factors)
        self.get_logger().info(f"Max distance {np.max(grasp_factors)}")
        grasp_factors /= np.max(grasp_factors)
        grasp_factors = 1 - grasp_factors

        self.save_grasps_in_polar(gg, save_folder, "Custom Stability Scores", c=grasp_factors)
        gg.scores = grasp_factors * (1 - gg.scores)
        self.save_grasps_in_polar(gg, save_folder, "Final Calculated Grasp Score", c=gg.scores)

        grippers = []
        for grasp in gg:
            gripper: Any = grasp.to_open3d_geometry(color=plt.get_cmap('Greens')(grasp.score)[:-1])
            grippers.append(gripper)
        centroid = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=geometric_centroid)
        o3d.visualization.draw_geometries([*grippers, cloud, centroid], window_name=f"All Grasp Score ")

        grippers = []
        for i, grasp in enumerate(gg):
            gripper: Any = grasp.to_open3d_geometry(color=plt.get_cmap('Greens')(grasp_factors[i])[:-1])
            grippers.append(gripper)
        centroid = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=geometric_centroid)
        o3d.visualization.draw_geometries([*grippers, cloud, centroid], window_name=f"Sideways Grasp Score")

        grippers = []
        for i, grasp in enumerate(gg):
            gripper: Any = grasp.to_open3d_geometry(color=plt.get_cmap('Greens')(lambdas[i])[:-1])
            grippers.append(gripper)
        centroid = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=geometric_centroid)
        o3d.visualization.draw_geometries([*grippers, cloud, centroid], window_name=f"Lambda Grasp Score")

    def generate_candidates_(self, x: float, y: float, z: float, radius: float, save_folder: str) -> tuple[GraspNetGroup | None, o3d.cuda.pybind.geometry.PointCloud | None]:
        """
        Generates the candidate set of grasps

        Parameters:
            x,y,z: the centre of the object
            radius: the bounding box radius of the object
            save_folder: the main directory of the data being stored
        Returns:
            a tuple of the grasp group and cloud, or None, None
        """
        
        # STEP 1: Retrieve Point Cloud
        points: npt.NDArray[np.float64]
        colours: npt.NDArray[np.float64]
        points, colours = self.retrieve_pointcloud(save_folder)

        # STEP 2: Crop Point Cloud
        cropped_points: npt.NDArray[np.float64] | None
        cropped_colours: npt.NDArray[np.float64] | None
        cropped_points, cropped_colours = self.crop_point_cloud(points, colours, x, y, z, radius, save_folder)
        if cropped_points is None or cropped_colours is None:
            return None, None
        
        # create rainbow cloud
        rainbow_cloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
        rainbow_cloud.points = o3d.utility.Vector3dVector(cropped_points)

        # STEP 3: Get Grasps
        t0: float = time.time()
        gg: GraspNetGrasp
        cloud: o3d.cuda.pybind.geometry.PointCloud
        if not self.OVERRIDE_ANYGRASP:
            gg, cloud = self.anygrasp.get_grasp(
                cropped_points.astype(np.float32), cropped_colours.astype(np.float32), 
                lims=self.anygrasp_lims, 
                apply_object_mask=self.APPLY_OBJECT_MASK, 
                dense_grasp=self.DENSE_GRASP, 
                collision_detection=self.APPLY_COLLISIONS
                )
        else:
            # we have a few npy's: 
                # one is all_grasps.npy --> this is nms and nothing else
                # another is filtered_grasps.npy --> this is after filtering and symmetry
                # the last is post-symmetry-grasps.npy --> this is after symmetry but no filtering
            gg = GraspNetGroup()
            gg.from_npy(f"{self.OVERRIDE_ANYGRASP_FOLDER}/anygrasp_output_grasps.npy")
            cloud = rainbow_cloud
        t1: float = time.time()

        # logging and error handling
        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "w") as f:
                f.write(f"Grasp algorithm time: {t1 - t0}\r\n")
            # saves data in the format (score, width, height, depth, rotation matrix (9 of them), translation (3 of them), object id)
            # there is also the from_npy function
            gg.save_npy(f"{save_folder}/grasp_groups/anygrasp_output_grasps")

        if gg is None or len(gg) == 0:
            self.get_logger().error('No Grasp detected after collision detection!')
            return None, None

        # for logging only
        unfiltered_gg: GraspNetGroup = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        )

        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Total num grasps: {len(gg)}\r\n")
                f.write(f"Total num grasps after nms: {len(unfiltered_gg)}\r\n")
            unfiltered_gg.save_npy(f"{save_folder}/grasp_groups/all_grasps_postnms")
            self.save_grasps_in_polar(gg, save_folder, "all")
            self.save_grasps_in_polar(unfiltered_gg, save_folder, "all_postnms")
        if self.VISUALISE:
            display_grasps(unfiltered_gg, rainbow_cloud, origin_position=[x,y,z], description="All Grasps")

        # STEP 6: Filter Grasps
        self.filter_grasps(gg)

        if len(gg) == 0:
            self.get_logger().error('No Grasps obtained after orientation filtering performed')
            return None, None
        
        if self.SAVE_DATA:
            self.save_grasps_in_polar(gg, save_folder, "post_filtering")
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Filtered num grasps: {len(gg)}\r\n")

        if self.SAVE_DATA:
            tmp_gg = gg.nms(
                translation_thresh = self.NMS_TRANSLATION_THRESH,
                rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
            ).sort_by_score()

            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Filtered num grasps after nms: {len(gg)}\r\n")
            gg.save_npy(f"{save_folder}/grasp_groups/filtered_grasps")
            self.save_grasps_in_polar(gg, save_folder, "post_filtering_postnms")

        # visualization
        if self.VISUALISE:
            display_grasps(gg, cloud, origin_position=[x,y,z], description="Filtered Grasps")

        # STEP 4: Compute Symmetries
        self.apply_symmetries(gg, cloud, x, y, z, save_folder)

        # STEP 5: TODO Recheck Collisions

        # STEP 7: Perform NMS
        gg = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        ).sort_by_score()

        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Num grasps after filtering_nms_symmetry: {len(gg)}\r\n")
            gg.save_npy(f"{save_folder}/grasp_groups/filtering_nms_symmetry")
            self.save_grasps_in_polar(gg, save_folder, "post_filtering_postnms_symmetry")

        # STEP 8: Correct Grasp Score
        if self.STABILITY_SCORE_CORRECTION_ENABLE:
            self.correct_scores(gg, cloud, save_folder)
            gg = gg.sort_by_score()

        # visualization
        if self.VISUALISE:
            display_grasps(gg, cloud, origin_position=[x,y,z], description="Candidate Grasps")
            display_grasps(gg, cloud, only_first=True, origin_position=[x,y,z], description="Highest Grasp Score")

        return gg, cloud

    def log_anygrasp_data(self, f: Any) -> None:
        """Logs config data"""
        f.write(f"z-coords point cloud bounding [{self.Z_COORDS_MIN}, {self.Z_COORDS_MAX}]\r\n")
        f.write(f"grasping bounds [{self.X_GRASP_MIN},{self.X_GRASP_MAX},{self.Y_GRASP_MIN},{self.Y_GRASP_MAX},{self.Z_GRASP_MIN},{self.Z_GRASP_MAX}]\r\n")
        f.write(f"apply: object mask [{self.APPLY_OBJECT_MASK}] collisions [{self.APPLY_COLLISIONS}] dense_grasp [{self.DENSE_GRASP}]\r\n")
        f.write(f"NMS thresholds: translation [{self.NMS_TRANSLATION_THRESH}] rotation [{self.NMS_ANGLE_THRESH_DEG}]\r\n")
        f.write(f"max pitch/roll filtering: [{self.MAX_GRASP_PITCH_ROLL_DEG}] degrees\r\n")
        f.write(f"grasp offsets: approach axis [{self.GRASP_AXIS_OFFSET}] vertical [{self.GRASP_Z_OFFSET}]\r\n")
        f.write(f"symmetry enabled? [{self.SYMMETRY_ENABLE}] layer height [{self.SYMMETRY_LAYER_HEIGHT}] rotation step [{self.SYMMETRY_ROTATION_STEP}] similarity thresh [{self.SYMMETRY_SIMILARITY_THRESHOLD}]\r\n")
        f.write(f"acquisition function eps [{self.ACQUISITION_EPS}] kappa [{self.ACQUISITION_KAPPA}] decay? [{self.ACQUISITION_ENABLE_DECAY}] rate [{self.ACQUISITION_EPS_DECAY_RATE}] final [{self.ACQUISITION_EPS_FINAL}]\r\n")
        f.write(f"kernel length scale z [{self.KERNEL_LENGTH_SCALE_Z}] theta [{self.KERNEL_LENGTH_SCALE_TH}] matern nu [{self.KERNEL_MATERN_NU}]\r\n")
        f.write(f"planning time multiplier [{self.PLANNING_TIME_MULTIPLIER}] base planning time [{self.BASELINE_PLANNING_TIME}] total planning time [{self.TOTAL_PLANNING_TIME_SEC}]\r\n")
        f.write(f"stabe score correction enabled? [{self.STABILITY_SCORE_CORRECTION_ENABLE}]\r\n")
        f.write(f"using point cloud file [{self.POINTCLOUD_FILE}]\r\n")

    def to_hts_gg(self, gg: GraspNetGroup, folder: str, flip_z: bool=False) -> HTSGraspGroup:
        """
        Converts a grasp group to a HTSGraspGroup.
        
        Parameters:
            gg: the grasp group
            folder: the main directory to save data to
            flip_z: whether to flip the grasp group or not

        Returns:
            a new HTSGraspGroup
        """

        hts_grasp_group: HTSGraspGroup = HTSGraspGroup()
        for ind, grasp in enumerate(gg):            
            hts_grasp = HTSGrasp(grasp, ind)
            hts_grasp.set_pose(self.map_grasp(grasp, flip_z=flip_z))

            if self.SAVE_DATA:
                hts_grasp.save_grasp_info(folder)

            hts_grasp_group.append(hts_grasp)

        return hts_grasp_group

    def grasp_callback_(self, goal_handle) -> RequestGrasp.Result:
        """Callback for grasp request"""
        # if self.GLOBAL_ITERATOR < 5: # base 
        #     self.ENABLE_GRASP_SELECTION = False
        #     self.PLOT_SELECTION_GRAPHS = False
        #     self.SYMMETRY_ENABLE = False
        #     self.STABILITY_SCORE_CORRECTION_ENABLE = False
        # elif self.GLOBAL_ITERATOR < 10: # symmetry
        #     self.ENABLE_GRASP_SELECTION = False
        #     self.PLOT_SELECTION_GRAPHS = False
        #     self.SYMMETRY_ENABLE = True
        #     self.STABILITY_SCORE_CORRECTION_ENABLE = False
        # elif self.GLOBAL_ITERATOR < 15: # corrected
        #     self.ENABLE_GRASP_SELECTION = False
        #     self.PLOT_SELECTION_GRAPHS = False
        #     self.SYMMETRY_ENABLE = True
        #     self.STABILITY_SCORE_CORRECTION_ENABLE = True
        # elif self.GLOBAL_ITERATOR < 20:
        #     self.ENABLE_GRASP_SELECTION = True
        #     self.PLOT_SELECTION_GRAPHS = True
        #     self.SYMMETRY_ENABLE = True
        #     self.STABILITY_SCORE_CORRECTION_ENABLE = True

        # self.GLOBAL_ITERATOR += 1

        request: RequestGrasp.Request = goal_handle.request
        feedback: RequestGrasp.Feedback = RequestGrasp.Feedback()
        response: RequestGrasp.Result = RequestGrasp.Result()
            
        folder: str = f"/ros2_ws/src/out/{time.time()}"

        # save data
        if self.SAVE_DATA:
            os.makedirs(folder)            
            os.makedirs(f"{folder}/plts")
            os.makedirs(f"{folder}/grasp_groups")
            with open(f"{folder}/info.txt", "a") as f:
                f.write(f"Request: Object ID {request.id} | Centred At ({request.x}, {request.y}, {request.z}) | Target ({request.goal_x},{request.goal_y},{request.goal_z})\r\n")
                self.log_anygrasp_data(f)

        # check if point cloud is valid
        if not self.POINTCLOUD_FROM_FILE and self.depth_pointcloud_ is None:
            self.get_logger().error("PointCloud Not Available")
            response.success = False
            response.message = "Point cloud not available"
            goal_handle.abort()
            return response

        # get candidates
        gg: GraspNetGroup | None
        cloud: o3d.cuda.pybind.geometry.PointCloud
        gg, cloud = self.generate_candidates_(request.x, request.y, request.z, self.MASK_RADIUS, folder)

        # check if gg is valid
        if gg is None or len(gg) == 0:
            self.get_logger().error("Grasp Failed")
            response.success = False
            response.message = "Unable to identify any grasps"
            goal_handle.abort()
            return response
        
        # publish feedback
        feedback.progress = f"Identified {len(gg)} grasps. Evaluating efficiency..."
        goal_handle.publish_feedback(feedback)

        hts_grasp_group: HTSGraspGroup = self.to_hts_gg(gg, folder, flip_z=False)
        hts_grasp_group_flipped: HTSGraspGroup = self.to_hts_gg(gg, folder, flip_z=True)

        tuner = LognormalPlanningTimeModel(
            planning_multiplier=self.PLANNING_TIME_MULTIPLIER,
            initial_planning_time=self.BASELINE_PLANNING_TIME,
            initial_var=0.5
        )

        tuner_move = LognormalPlanningTimeModel(
            planning_multiplier=self.PLANNING_TIME_MULTIPLIER,
            initial_planning_time=self.BASELINE_PLANNING_TIME_MOVE,
            initial_var=0.5
        )

        self.get_logger().info(f"Tuner initial planning mean {tuner.planning_time_mean} time {self.BASELINE_PLANNING_TIME}")
        self.get_logger().info(f"Tuner initial planning mean {tuner_move.planning_time_mean} time {self.BASELINE_PLANNING_TIME_MOVE}")


        acquisition_fn = EpsilonGreedyUCB(
            kappa=self.ACQUISITION_KAPPA,
            eps=self.ACQUISITION_EPS,
            eps_final=self.ACQUISITION_EPS_FINAL if self.ACQUISITION_ENABLE_DECAY else None,
            eps_decay_rate=self.ACQUISITION_EPS_DECAY_RATE
        )

        context = ValidityContext(goal_handle=goal_handle, grasp_group=hts_grasp_group, folder=folder, cloud=cloud,
            request=request, plot=self.PLOT_SELECTION_GRAPHS, visualise=self.VISUALISE, logger=self.get_logger(), is_flipped=False,
            save_data=True, client=self.grasp_validity_client_
        )

        context_flipped = ValidityContext(goal_handle=goal_handle, grasp_group=hts_grasp_group_flipped, folder=folder, cloud=cloud,
            request=request, plot=self.PLOT_SELECTION_GRAPHS, visualise=self.VISUALISE, logger=self.get_logger(), is_flipped=True,
            save_data=True, client=self.grasp_validity_client_
        )

        if self.ENABLE_GRASP_SELECTION:
            final_context = ValidityContext(goal_handle=goal_handle)

            first_selector = GPGraspSelector(
                hts_grasp_group, tuner, tuner_move, acquisition_fn, context,
                length_scale_z=self.KERNEL_LENGTH_SCALE_Z, 
                matern_nu_z=self.KERNEL_MATERN_NU, 
                length_scale_th=self.KERNEL_LENGTH_SCALE_TH,
                total_planning_time=self.TOTAL_PLANNING_TIME_SEC
            )
            second_selector = GPGraspSelector(
                hts_grasp_group_flipped, tuner, tuner_move, acquisition_fn, context_flipped,
                length_scale_z=self.KERNEL_LENGTH_SCALE_Z, 
                matern_nu_z=self.KERNEL_MATERN_NU, 
                length_scale_th=self.KERNEL_LENGTH_SCALE_TH,
                total_planning_time=self.TOTAL_PLANNING_TIME_SEC
            )
            dual_problem = DualGPGraspSelector(first_selector, second_selector, final_context)
            dual_problem._handle_validity_send_goal()

            while final_context.response is None:
                time.sleep(0.01)
            time.sleep(1.0)

            if final_context.response.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return final_context.response
        else:
            problem = SequentialGraspSelector(hts_grasp_group, tuner, tuner_move, SequentialAcquisition(), context)
            problem._handle_validity_send_goal()

            while context.response is None:
                time.sleep(0.01)
            time.sleep(1.0)

            problem = SequentialGraspSelector(hts_grasp_group_flipped, tuner, tuner_move, SequentialAcquisition(), context_flipped)
            problem._handle_validity_send_goal()

            while context_flipped.response is None:
                time.sleep(0.01)
            time.sleep(1.0)

            if context.response.success:
                response = context.response
                goal_handle.succeed()
                return response
            elif context_flipped.response.success:
                response = context_flipped.response
                goal_handle.succeed()
                return response
            else:
                response = context.response
                goal_handle.abort()
                return response

    def map_grasp(self, grasp: GraspNetGrasp, flip_z: bool=False) -> Pose:
        """
        Maps a grasp to the problem domain

        Parameters:
            grasp: the grasp to map
            flip_z: whether to flip the gripper to be facing down or not

        Returns:
            the resulting pose
        """

        # the original grasp rotation
        grasp_rotation : Rotation = Rotation.from_matrix(grasp.rotation_matrix)

        # define offset and flip rotations
        offset_rotation: Rotation = Rotation.from_euler('y', 90, degrees=True)
        flip_rotation: Rotation = Rotation.from_euler('z', 180, degrees=True)

        # calculates the final rotation
        final_rotation: Rotation = grasp_rotation * offset_rotation

        # detect if camera is pointing downwards
        x_axis: npt.NDArray[np.float64] = final_rotation.as_matrix()[:, 0] # x_axis[2] > 0 ==> camera is facing upwards
        if (x_axis[2] < 0) ^ flip_z: # we are up and want to be down, or vice versa
            final_rotation =  final_rotation * flip_rotation

        # local axes:
            # z points in the direction of grasp attack
            # y is perpendicular to z in the horizontal plane (action of gripper)
            # x points vertical

        # calculate the offset and final translatiosn
        offset_translation: npt.NDArray[np.float64] = np.array([0, 0, -self.GRASP_AXIS_OFFSET])
        grasp.translation[2] += self.GRASP_Z_OFFSET
        
        # calculate final pose
        final_translation: npt.NDArray[np.float64] = grasp.translation + final_rotation.as_matrix() @ offset_translation
        final_quaternion: Any = final_rotation.as_quat()

        pose: Pose = Pose()
        pose.position.x = final_translation[0]
        pose.position.y = final_translation[1]
        pose.position.z = final_translation[2]
        pose.orientation.x = final_quaternion[0]
        pose.orientation.y = final_quaternion[1]
        pose.orientation.z = final_quaternion[2]
        pose.orientation.w = final_quaternion[3]

        return pose

    def save_grasps_in_polar(self, gg: GraspNetGroup, folder: str, descr: str="", c: npt.NDArray[np.float64] | None=None) -> None:
        """
        Saves a polar map of a grasp group

        Parameters:
            gg: the grasp group
            folder: the main directory to save data to
            descr: a description of the plot
            c: an array of colours, or None        
        """

        hts_grasp_group: HTSGraspGroup = HTSGraspGroup()

        for ind, grasp in enumerate(gg):            
            hts_grasp = HTSGrasp(grasp, ind)
            hts_grasp.set_pose(self.map_grasp(grasp, flip_z=False))
            hts_grasp_group.append(hts_grasp)
        
        zs: npt.NDArray[np.float64] = np.array([p.z for p in hts_grasp_group._grasps]).reshape((-1, 1))
        ths: npt.NDArray[np.float64] = np.array([p.theta for p in hts_grasp_group._grasps]).reshape((-1, 1))

        fig = plt.figure()
        ax = fig.add_subplot(projection="polar")
        plot = ax.scatter(ths, zs, c=c)
        ax.set_title(f"Grasp Group: {descr}")
        if c is not None:
            fig.colorbar(plot, ax=ax)
        fig.savefig(f"{folder}/grasp_group_{descr}.png", format="png")
        plt.close(fig)

    def create_symmetry_grasps(self, gg: GraspNetGroup, cloud: o3d.cuda.pybind.geometry.PointCloud) -> None:
        """
        Applies symmetry generation to a grasp group

        Parameters:
            gg: the grasp group
            cloud: the point cloud 
        """

        # batch grasps in horizontal layers
        layer_base_height: float = max(self.Z_COORDS_MIN, self.Z_GRASP_MIN)
        while (layer_base_height < min(self.Z_COORDS_MAX, self.Z_GRASP_MAX)):
            self.get_logger().info(f"Slicing layer at height {layer_base_height}")

            # filter cloud and grasps by height
            bb: Any = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=[-math.inf, -math.inf, layer_base_height],
                max_bound=[math.inf, math.inf, layer_base_height + self.SYMMETRY_LAYER_HEIGHT]
            )
            layer_cloud: o3d.cuda.pybind.geometry.PointCloud = cloud.crop(bb)
            layer_grasps: list[GraspNetGrasp] = [grasp for grasp in gg if grasp.translation[2] >= layer_base_height and grasp.translation[2] < layer_base_height + self.SYMMETRY_LAYER_HEIGHT]
            
            if layer_cloud.is_empty() or len(layer_grasps) == 0:
                self.get_logger().info(f"Skipping... {layer_cloud} {len(layer_grasps)}")

                layer_base_height += self.SYMMETRY_LAYER_HEIGHT
                continue

            group: SymmetryGroup = SymmetryGroup(layer_cloud, layer_grasps)
            
            # perform rotations around the centre
            for i in range(self.SYMMETRY_ROTATION_STEP, 360, self.SYMMETRY_ROTATION_STEP):
                self.get_logger().info(f"Performing rotation {i} degrees")
                new_group = group.rotate_about_centre(i)
                if new_group.clouds_are_similar(group, self.SYMMETRY_SIMILARITY_THRESHOLD):
                    self.get_logger().info(f"Adding {len(new_group.grasps())} grasps")
                    gg.add(new_group.grasps())
                else:
                    self.get_logger().info("Clouds are not similar")

            layer_base_height += self.SYMMETRY_LAYER_HEIGHT

def main():
    rclpy.init(args=None)

    node = AnyGraspNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("Stopped AnyGrasp Node")

if __name__ == '__main__':
    main()