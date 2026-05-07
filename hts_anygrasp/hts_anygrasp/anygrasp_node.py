from __future__ import annotations
import os
import time
import argparse
import math
import numpy as np
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

pkg_prefix = get_package_prefix("hts_anygrasp")
lib_path = os.path.join(pkg_prefix, "lib", "hts_anygrasp")
os.environ["LD_LIBRARY_PATH"] = (lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", ""))
checkpoint_path = os.path.join(pkg_prefix, "share/hts_anygrasp/checkpoint_detection.tar")

from graspnetAPI.grasp import Grasp as GraspNetGrasp
from graspnetAPI import GraspGroup as GraspNetGroup
from gsnet import AnyGrasp

from .hts_grasps import HTSGrasp, HTSGraspGroup
from .symmetry import SymmetryGroup

# from .grasp_selection_basic import GraspSelectorBasic
# from .grasp_selection_gp import GraspSelectorGP, DualGraspSelectorGP, PlanningTimeTuner
from .utils import display_grasps, display_pointcloud, fast_norgb_pc2_to_numpy, fast_pc2_to_numpy, ValidityContext
from .grasp_selection import DualGPGraspSelector, GPGraspSelector, EpsilonGreedyUCB, LognormalPlanningTimeModel, SequentialGraspSelector, SequentialAcquisition

class AnyGraspNode(Node):
    def __init__(self):
        super().__init__('hts_anygrasp')

        # DECLARE PARAMETERS
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
        self.declare_parameter('acquisition_kappa', 3.0)
        self.declare_parameter('acquisition_eps', 0.1)
        self.declare_parameter('kernel_length_scale_z', 0.03)
        self.declare_parameter('kernel_length_scale_th', 0.8)
        self.declare_parameter('kernel_matern_nu', 2.5)
        self.declare_parameter('total_planning_time_sec', 300)
        self.declare_parameter('acquisition_enable_decay', False)
        self.declare_parameter('acquisition_eps_final', 0.1)
        self.declare_parameter('acquisition_eps_decay_rate', 0.99)

        # config options for point/grasp bounding
        self.Z_COORDS_MIN: float = self.get_parameter('z_coords_min').value
        self.Z_COORDS_MAX: float = self.get_parameter('z_coords_max').value
        self.X_GRASP_MIN: float = self.get_parameter('x_grasp_min').value 
        self.X_GRASP_MAX: float = self.get_parameter('x_grasp_max').value
        self.Y_GRASP_MIN: float = self.get_parameter('y_grasp_min').value 
        self.Y_GRASP_MAX: float = self.get_parameter('y_grasp_max').value
        self.Z_GRASP_MIN: float = self.get_parameter('z_grasp_min').value 
        self.Z_GRASP_MAX: float = self.get_parameter('z_grasp_max').value
        self.MASK_RADIUS: str = self.get_parameter('mask_radius').value

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
        self.SYMMETRY_ROTATION_STEP: float = self.get_parameter('symmetry_rotation_step').value
        self.SYMMETRY_SIMILARITY_THRESHOLD: float = self.get_parameter('symmetry_similarity_threshold').value

        # pointcloud to listen on
        self.depth_pointcloud_: PointCloud2 = None

        # config for grasp selection
        self.ENABLE_GRASP_SELECTION: bool = self.get_parameter('enable_grasp_selection').value      
        self.PLOT_SELECTION_GRAPHS: bool = self.get_parameter('plot_selection_graphs').value
        self.PLANNING_TIME_MULTIPLIER: float = self.get_parameter('planning_time_multiplier').value
        self.BASELINE_PLANNING_TIME: float = self.get_parameter('baseline_planning_time').value
        self.ACQUISITION_KAPPA: float = self.get_parameter('acquisition_kappa').value
        self.ACQUISITION_EPS: float = self.get_parameter('acquisition_eps').value
        self.KERNEL_LENGTH_SCALE_Z: float = self.get_parameter('kernel_length_scale_z').value
        self.KERNEL_LENGTH_SCALE_TH: float = self.get_parameter('kernel_length_scale_th').value
        self.KERNEL_MATERN_NU: float = self.get_parameter('kernel_matern_nu').value
        self.TOTAL_PLANNING_TIME_SEC: float = self.get_parameter('total_planning_time_sec').value
        self.ACQUISITION_ENABLE_DECAY: bool = self.get_parameter('acquisition_enable_decay').value
        self.ACQUISITION_EPS_FINAL: float = self.get_parameter('acquisition_eps_final').value
        self.ACQUISITION_EPS_DECAY_RATE: float = self.get_parameter('acquisition_eps_decay_rate').value

        # load in point cloud from file if necessary
        if self.POINTCLOUD_FROM_FILE:
            from_file = np.load(self.POINTCLOUD_FILE)
            self.file_points = from_file['points']
            if not self.NO_RGB:
                self.file_colors = from_file['colors']

        # configs for anygrasp
        cfgs = argparse.Namespace(
            checkpoint_path=checkpoint_path,
            max_gripper_width=max(0, min(0.1, self.MAX_GRIPPER_WIDTH)),
            gripper_height=self.GRIPPER_HEIGHT,
            top_down_grasp=self.TOP_DOWN_GRASP,
            debug=True, # was true
        )
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

    def pointcloud_callback_(self, msg):
        self.depth_pointcloud_ = msg

    def display_callback_(self, request, response):
        # Convert PointCloud2 to numpy array
        self.get_logger().info("Displaying...")
        if self.depth_pointcloud_ is None:
            self.get_logger().info("Found no points")

        if self.NO_RGB:
            points, colors = fast_norgb_pc2_to_numpy(self.depth_pointcloud_)
        else:
            points, colors = fast_pc2_to_numpy(self.depth_pointcloud_)

        if points.shape[0] == 0:
            self.get_logger().info("No points found")
            return response

        # Open3D visualization
        display_pointcloud(points, colors)

        if self.SAVE_DATA:
            np.savez(f"src/pointclouds/displayed_cloud_{time.time()}.npz", points=points, colors=colors)

        return response

    def retrieve_pointcloud(self, save_folder):
        if self.NO_RGB:
            if not self.POINTCLOUD_FROM_FILE:
                points, colors = self.fast_norgb_pc2_to_numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colors = np.zeros_like(points, dtype=np.float32)
            if self.VISUALISE:
                display_pointcloud(points, save=self.SAVE_DATA, filename=f"{save_folder}/full_cloud", description="Full Point Cloud")
        else:
            if not self.POINTCLOUD_FROM_FILE:
                points, colors = self.fast_pc2_to_numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colors = self.file_colors
            if self.VISUALISE:
                display_pointcloud(points, colors, save=self.SAVE_DATA, filename=f"{save_folder}/full_cloud", description="Full Point Cloud")

        return points, colors

    def crop_point_cloud(self, points, colors, x, y, z, radius, save_folder):
        # filter according to z
        z_coords = points[:, 2]
        y_coords = points[:, 1]
        x_coords = points[:, 0]
        mask = (z_coords > self.Z_COORDS_MIN) & (z_coords < self.Z_COORDS_MAX) & ((x - x_coords)**2 + (y - y_coords)**2 < radius**2)
        
        cropped_points = points[mask].astype(np.float32)
        cropped_colors = colors[mask].astype(np.float32)
        uncropped_points = points[~mask].astype(np.float32)
        uncropped_colors = colors[~mask].astype(np.float32)

        if not cropped_points.shape[0]:
            self.get_logger().error("Cropped pointcloud contains no points")
            return None, None
        
        # show cropped and uncropped pointclouds
        if self.VISUALISE:
            if self.NO_RGB:
                display_pointcloud(cropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                display_pointcloud(uncropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")
            else:
                display_pointcloud(cropped_points, cropped_colors, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                display_pointcloud(uncropped_points, uncropped_colors, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")

        return cropped_points, cropped_colors

    def apply_symmetries(self, gg, cloud, x, y, z, save_folder):
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

    def filter_grasps(self, gg):
        exclude_grasps = []
        for ind, grasp in enumerate(gg):
            # exclude grasps by width
            if grasp.width < self.MIN_GRASP_WIDTH:
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

    def generate_pose_(self, x, y, z, radius, save_folder):
        # STEP 1: Retrieve Point Cloud
        points, colors = self.retrieve_pointcloud(save_folder)


        # STEP 2: Crop Point Cloud
        cropped_points, cropped_colors = self.crop_point_cloud(points, colors, x, y, z, radius, save_folder)
        if cropped_points is None or cropped_colors is None:
            return None, None
        
        # create rainbow cloud
        rainbow_cloud = o3d.geometry.PointCloud()
        rainbow_cloud.points = o3d.utility.Vector3dVector(cropped_points)

        # STEP 3: Get Grasps
        t0 = time.time()
        gg, cloud = self.anygrasp.get_grasp(
            cropped_points, cropped_colors, 
            lims=self.anygrasp_lims, 
            apply_object_mask=self.APPLY_OBJECT_MASK, 
            dense_grasp=self.DENSE_GRASP, 
            collision_detection=self.APPLY_COLLISIONS
            )
        t1 = time.time()

        # logging and error handling
        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "w") as f:
                f.write(f"Grasp algorithm time: {t1 - t0}\r\n")

        if gg is None or len(gg) == 0:
            self.get_logger().error('No Grasp detected after collision detection!')
            return None, None
        
        # for logging only
        unfiltered_gg = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        )

        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Total num grasps: {len(gg)}\r\n")
                f.write(f"Total num grasps after nms: {len(unfiltered_gg)}\r\n")
            unfiltered_gg.save_npy(f"{save_folder}/grasp_groups/all_grasps")
            self.save_grasps_in_polar(gg, save_folder, "all")
            self.save_grasps_in_polar(unfiltered_gg, save_folder, "all_postnms")
        if self.VISUALISE:
            display_grasps(unfiltered_gg, rainbow_cloud, origin_position=[x,y,z], description="All Grasps")

        # STEP 4: Compute Symmetries
        self.apply_symmetries(gg, cloud, x, y, z, save_folder)

        # STEP 5: TODO Recheck Collisions

        # STEP 6: Filter Grasps
        self.filter_grasps(gg)

        if len(gg) == 0:
            self.get_logger().error('No Grasps obtained after orientation filtering performed')
            return
        
        if self.SAVE_DATA:
            self.save_grasps_in_polar(gg, save_folder, "post_filtering")
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Filtered num grasps: {len(gg)}\r\n")

        # STEP 7: Perform NMS
        gg = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        ).sort_by_score()

        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Filtered num grasps after nms: {len(gg)}\r\n")
            gg.save_npy(f"{save_folder}/grasp_groups/filtered_grasps")
            self.save_grasps_in_polar(gg, save_folder, "post_filtering_postnms")

        # STEP 8: Correct Grasp Score
        self.save_grasps_in_polar(gg, save_folder, "before correction", c=gg.scores)
        original_scores = gg.scores
        original_xyz = np.array([g.translation for g in gg])
        centroid = cloud.get_center()

        distance_above_centroid = original_xyz[:, 2] - centroid[2]
        max_distance = np.max(np.linalg.norm(original_xyz - centroid, axis=-1))

        stable_score = np.abs(distance_above_centroid)/(max_distance*1.5)

        self.save_grasps_in_polar(gg, save_folder, "stable score", c=stable_score)
        unstable_scores = (original_scores/(1 - stable_score)).clip(max=1.0)
        self.save_grasps_in_polar(gg, save_folder, "without stable score", c=unstable_scores)

        # simplify the structure to stacked rings
        total_mass = 0
        total_ixx = 0
        
        SLICE_LAYER_HEIGHT = 0.02

        layer_base_height = self.Z_COORDS_MIN
        while (layer_base_height < self.Z_COORDS_MAX):
            self.get_logger().info(f"Slicing layer at height {layer_base_height}")

            # filter cloud and grasps by height
            bb = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=[-math.inf, -math.inf, layer_base_height],
                max_bound=[math.inf, math.inf, layer_base_height + SLICE_LAYER_HEIGHT]
            )
            layer_cloud = cloud.crop(bb)

            if layer_cloud.is_empty():
                self.get_logger().info(f"Skipping... {layer_cloud}")
                layer_base_height += SLICE_LAYER_HEIGHT
                continue

            layer_points = np.asarray(layer_cloud.points)
            rsquared = np.square(layer_points[:,0] - centroid[0]) + np.square(layer_points[:,1] - centroid[1])
            average_rsquared= np.mean(rsquared)
            layer_mass = average_rsquared*np.pi*SLICE_LAYER_HEIGHT # alternatively, we could do convex hull
            total_mass += layer_mass

            layer_ixx = 1/4*layer_mass*average_rsquared + 1/3*layer_mass*(SLICE_LAYER_HEIGHT**2)
            shifted_ixx = layer_ixx + layer_mass*((centroid[2] - layer_base_height)**2)
            total_ixx += shifted_ixx
            layer_base_height += SLICE_LAYER_HEIGHT

        # now we have an estimate of Ixx and m
        lambdas = -np.sign(distance_above_centroid)*np.sqrt(total_mass*9.81*np.abs(distance_above_centroid)/(total_ixx + total_mass*distance_above_centroid**2))

        # anything that is negative is already stable
        if np.max(lambdas):
            lambdas = lambdas/np.abs(np.max(lambdas))

        self.save_grasps_in_polar(gg, save_folder, "lambdas", c=lambdas)

        # lambdas = lambdas.clip(min=0.0)

        # multiply grasp scores by this
        gg.scores = gg.scores*(1 - lambdas)
        self.save_grasps_in_polar(gg, save_folder, "corrected score", c=gg.scores)

        # visualization
        if self.VISUALISE:
            display_grasps(gg, cloud, origin_position=[x,y,z], description="Filtered Grasps")
            display_grasps(gg, cloud, only_first=True, origin_position=[x,y,z], description="Highest Grasp Score")

        return gg, cloud

    def log_anygrasp_data(self, f):
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
        f.write(f"using point cloud file [{self.POINTCLOUD_FILE}]\r\n")

    def grasp_callback_(self, goal_handle):
        request = goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()
            
        folder = f"/ros2_ws/src/out/{time.time()}"
        if self.SAVE_DATA:
            os.makedirs(folder)            
            os.makedirs(f"{folder}/plts")
            os.makedirs(f"{folder}/grasp_groups")
            with open(f"{folder}/info.txt", "a") as f:
                f.write(f"Request: Object ID {request.id} | Centred At ({request.x}, {request.y}, {request.z}) | Target ({request.goal_x},{request.goal_y},{request.goal_z})\r\n")
                self.log_anygrasp_data(f)

        if not self.POINTCLOUD_FROM_FILE and self.depth_pointcloud_ is None:
            self.get_logger().error("PointCloud Not Available")
            response.success = False
            response.message = "Point cloud not available"
            goal_handle.abort()
            return response

        gg, cloud = self.generate_pose_(request.x, request.y, request.z, self.MASK_RADIUS, folder)

        if gg is None or len(gg) == 0:
            self.get_logger().error("Grasp Failed")
            response.success = False
            response.message = "Unable to identify any grasps"
            goal_handle.abort()
            return response
        
        feedback.progress = f"Identified {len(gg)} grasps. Evaluating efficiency..."
        goal_handle.publish_feedback(feedback)

        hts_grasp_group = HTSGraspGroup()
        hts_grasp_group_mirrored = HTSGraspGroup()

        for ind, grasp in enumerate(gg):            
            hts_grasp = HTSGrasp(grasp, ind)
            hts_grasp.set_pose(self.map_grasp(grasp, flip_z=False))

            if self.SAVE_DATA:
                hts_grasp.save_grasp_info(folder)

            hts_grasp_group.append(hts_grasp)

        for ind, grasp in enumerate(gg):            
            hts_grasp = HTSGrasp(grasp, ind)
            hts_grasp.set_pose(self.map_grasp(grasp, flip_z=True))

            if self.SAVE_DATA:
                hts_grasp.save_grasp_info(folder)

            hts_grasp_group_mirrored.append(hts_grasp)

        tuner = LognormalPlanningTimeModel(
            planning_multiplier=self.PLANNING_TIME_MULTIPLIER,
            initial_planning_time=self.BASELINE_PLANNING_TIME,
            initial_var=0.5
        )

        acquisition_fn = EpsilonGreedyUCB(
            kappa=self.ACQUISITION_KAPPA,
            eps=self.ACQUISITION_EPS,
            eps_final=self.ACQUISITION_EPS_FINAL if self.ACQUISITION_ENABLE_DECAY else None,
            eps_decay_rate=self.ACQUISITION_EPS_DECAY_RATE
        )

        context = ValidityContext(
            goal_handle=goal_handle,
            grasp_group=hts_grasp_group,
            folder=folder,
            cloud=cloud,
            request=request,
            plot=self.PLOT_SELECTION_GRAPHS,
            visualise=self.VISUALISE,
            logger=self.get_logger(),
            is_flipped=False,
            save_data=True,
            client=self.grasp_validity_client_
        )

        context_flipped = ValidityContext(
            goal_handle=goal_handle,
            grasp_group=hts_grasp_group_mirrored,
            folder=folder,
            cloud=cloud,
            request=request,
            plot=self.PLOT_SELECTION_GRAPHS,
            visualise=self.VISUALISE,
            logger=self.get_logger(),
            is_flipped=True,
            save_data=True,
            client=self.grasp_validity_client_
        )

        if self.ENABLE_GRASP_SELECTION:
            final_context = ValidityContext(goal_handle=goal_handle)

            first_selector = GPGraspSelector(
                hts_grasp_group, tuner, acquisition_fn, context,
                length_scale_z=self.KERNEL_LENGTH_SCALE_Z, 
                matern_nu_z=self.KERNEL_MATERN_NU, 
                length_scale_th=self.KERNEL_LENGTH_SCALE_TH,
                total_planning_time=self.TOTAL_PLANNING_TIME_SEC
            )
            second_selector = GPGraspSelector(
                hts_grasp_group_mirrored, tuner, acquisition_fn, context_flipped,
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

            return final_context.response
        else:
            problem = SequentialGraspSelector(hts_grasp_group, tuner, SequentialAcquisition(), context)
            problem._handle_validity_send_goal()

            while context.response is None:
                time.sleep(0.01)
            time.sleep(1.0)

            problem = SequentialGraspSelector(hts_grasp_group_mirrored, tuner, SequentialAcquisition(), context_flipped)
            problem._handle_validity_send_goal()

            while context_flipped.response is None:
                time.sleep(0.01)
            time.sleep(1.0)

            context.goal_handle.succeed()
            response = RequestGrasp.Result()
            response.success = False
            return response

    def map_grasp(self, grasp, flip_z=False):
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

        offset_translation = np.array([0, 0, -self.GRASP_AXIS_OFFSET])
        grasp.translation[2] += self.GRASP_Z_OFFSET
        final_translation = grasp.translation + final_rotation.as_matrix() @ offset_translation
        final_quaternion = final_rotation.as_quat()

        pose = Pose()
        pose.position.x = final_translation[0]
        pose.position.y = final_translation[1]
        pose.position.z = final_translation[2]
        pose.orientation.x = final_quaternion[0]
        pose.orientation.y = final_quaternion[1]
        pose.orientation.z = final_quaternion[2]
        pose.orientation.w = final_quaternion[3]

        return pose

    def save_grasps_in_polar(self, gg, folder, descr="", c=None):
        hts_grasp_group = HTSGraspGroup()

        for ind, grasp in enumerate(gg):            
            hts_grasp = HTSGrasp(grasp, ind)
            hts_grasp.set_pose(self.map_grasp(grasp, flip_z=False))
            hts_grasp_group.append(hts_grasp)
        
        zs = np.array([p.z for p in hts_grasp_group._grasps]).reshape((-1, 1))
        ths = np.array([p.theta for p in hts_grasp_group._grasps]).reshape((-1, 1))

        fig = plt.figure()
        ax = fig.add_subplot(projection="polar")
        plot = ax.scatter(ths, zs, c=c)
        ax.set_title(f"Grasp Group: {descr}")
        if c is not None:
            fig.colorbar(plot, ax=ax)
        fig.savefig(f"{folder}/grasp_group_{descr}.png", format="png")
        plt.close(fig)

    def create_symmetry_grasps(self, gg, cloud):
        # batch grasps in horizontal layers
        layer_base_height = max(self.Z_COORDS_MIN, self.Z_GRASP_MIN)
        while (layer_base_height < min(self.Z_COORDS_MAX, self.Z_GRASP_MAX)):
            self.get_logger().info(f"Slicing layer at height {layer_base_height}")
            # filter cloud and grasps by height
            bb = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=[-math.inf, -math.inf, layer_base_height],
                max_bound=[math.inf, math.inf, layer_base_height + self.SYMMETRY_LAYER_HEIGHT]
            )
            layer_cloud = cloud.crop(bb)
            layer_grasps = [grasp for grasp in gg if grasp.translation[2] >= layer_base_height and grasp.translation[2] < layer_base_height + self.SYMMETRY_LAYER_HEIGHT]
            
            if layer_cloud.is_empty() or len(layer_grasps) == 0:
                self.get_logger().info(f"Skipping... {layer_cloud} {len(layer_grasps)}")

                layer_base_height += self.SYMMETRY_LAYER_HEIGHT
                continue

            group = SymmetryGroup(layer_cloud, layer_grasps, self.get_logger())
            
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