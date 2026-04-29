from __future__ import annotations
import os
import time
import argparse
import math
import numpy as np
import open3d as o3d
import time

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
from .grasp_selection_custom import GraspSelectorCustom, ValidityContext
from .grasp_selection_basic import GraspSelectorBasic
from .grasp_selection_gp import GraspSelectorGP
from .utils import display_grasps, display_pointcloud, fast_norgb_pc2_to_numpy, fast_pc2_to_numpy


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

    def generate_pose_(self, x, y, z, radius, save_folder):
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
            return None

        # show cropped and uncropped pointclouds
        if self.VISUALISE:
            if self.NO_RGB:
                display_pointcloud(cropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                display_pointcloud(uncropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")
            else:
                display_pointcloud(cropped_points, cropped_colors, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                display_pointcloud(uncropped_points, uncropped_colors, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")

        # set workspace to filter output grasps
        xmin, xmax = self.X_GRASP_MIN, self.X_GRASP_MAX
        ymin, ymax = self.Y_GRASP_MIN, self.Y_GRASP_MAX
        zmin, zmax = self.Z_GRASP_MIN, self.Z_GRASP_MAX
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        t0 = time.time()
        gg, cloud = self.anygrasp.get_grasp(
            cropped_points, cropped_colors, 
            lims=lims, 
            apply_object_mask=self.APPLY_OBJECT_MASK, 
            dense_grasp=self.DENSE_GRASP, 
            collision_detection=self.APPLY_COLLISIONS
            )
        t1 = time.time()

        # replace cloud with rainbow point clouds
        rainbow_cloud = o3d.geometry.PointCloud()
        rainbow_cloud.points = o3d.utility.Vector3dVector(cropped_points)
        
        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "w") as f:
                f.write(f"Grasp algorithm time: {t1 - t0}\r\n")

        if gg is None or len(gg) == 0:
            self.get_logger().error('No Grasp detected after collision detection!')
            return None
        
        unfiltered_gg = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        )

        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Total num grasps: {len(gg)}\r\n")
                f.write(f"Total num grasps after nms: {len(unfiltered_gg)}\r\n")
            unfiltered_gg.save_npy(f"{save_folder}/grasp_groups/all_grasps")
        if self.VISUALISE:
            display_grasps(unfiltered_gg, rainbow_cloud, origin_position=[x,y,z], description="All Grasps")

        # compute symmetries
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
            if self.VISUALISE:
                display_grasps(unfiltered_gg, cloud, origin_position=[x,y,z], description="Post-Symmetry Grasps")

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

        if len(gg) == 0:
            self.get_logger().error('No Grasps obtained after orientation filtering performed')
            return
        
        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Filtered num grasps: {len(gg)}\r\n")

        # perform non-maximum suppression
        gg = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        ).sort_by_score()

        if self.SAVE_DATA:
            with open(f"{save_folder}/grasp_metrics.txt", "a") as f:
                f.write(f"Filtered num grasps after nms: {len(gg)}\r\n")
            gg.save_npy(f"{save_folder}/grasp_groups/filtered_grasps")

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

        context = ValidityContext(goal_handle, hts_grasp_group, folder, cloud, request)
        problem = GraspSelectorGP(hts_grasp_group, self.get_logger(), self.grasp_validity_client_)
        problem.start_timer()
        problem._handle_validity_send_goal(context)

        while context.response is None:
            time.sleep(0.01)
        time.sleep(1.0)

        context_flipped = ValidityContext(goal_handle, hts_grasp_group_mirrored, folder, cloud, request, is_flipped=True)
        problem_flipped = GraspSelectorGP(hts_grasp_group_mirrored, self.get_logger(), self.grasp_validity_client_)
        problem_flipped.start_timer()
        problem_flipped._handle_validity_send_goal(context_flipped)

        while context_flipped.response is None:
            time.sleep(0.01)
        time.sleep(1.0)

        return context.response
    
    def map_grasp(self, grasp, flip_z=False):
        grasp_rotation = Rotation.from_matrix(grasp.rotation_matrix)
        offset_rotation = Rotation.from_euler('y', 90, degrees=True)
        final_rotation = grasp_rotation * offset_rotation

        # local axes:
            # z points in the direction of grasp attack
            # y is perpendicular to z in the horizontal plane
            # x points vertical

        offset_translation = np.array([0, 0, -self.GRASP_AXIS_OFFSET])
        grasp.translation[2] += self.GRASP_Z_OFFSET
        final_translation = grasp.translation + final_rotation.as_matrix() @ offset_translation

        # detect if camera is pointing downwards
        flip_rotation = Rotation.from_euler('x', 180, degrees=True)
        roll, pitch, yaw = final_rotation.as_euler('xyz', degrees=True)
        # self.get_logger().info(f"Identified Grasp with roll {roll} pitch {pitch} yaw {yaw}. Unflipping...")
        if flip_z ^ (abs(roll) > 90):
            self.get_logger().info(f"Identified Grasp with roll {roll} pitch {pitch} yaw {yaw}. Unflipping...")
            final_rotation = final_rotation * flip_rotation
            nroll, npitch, nyaw = final_rotation.as_euler('xyz', degrees=True)
            self.get_logger().info(f"Now Grasp with roll {nroll} pitch {npitch} yaw {nyaw}.")

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