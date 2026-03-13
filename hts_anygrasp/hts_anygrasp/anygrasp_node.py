import os
import time
import argparse
import math
import matplotlib.cm as cm
import threading
import numpy as np
import open3d as o3d

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

from graspnetAPI import GraspGroup
from gsnet import AnyGrasp

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

        self.Z_COORDS_MIN = self.get_parameter('z_coords_min').value
        self.Z_COORDS_MAX = self.get_parameter('z_coords_max').value
        self.X_GRASP_MIN = self.get_parameter('x_grasp_min').value 
        self.X_GRASP_MAX = self.get_parameter('x_grasp_max').value
        self.Y_GRASP_MIN = self.get_parameter('y_grasp_min').value 
        self.Y_GRASP_MAX = self.get_parameter('y_grasp_max').value
        self.Z_GRASP_MIN = self.get_parameter('z_grasp_min').value 
        self.Z_GRASP_MAX = self.get_parameter('z_grasp_max').value
        self.APPLY_OBJECT_MASK = self.get_parameter('apply_object_mask').value
        self.APPLY_COLLISIONS = self.get_parameter('apply_collisions').value
        self.DENSE_GRASP = self.get_parameter('dense_grasp').value
        self.MIN_GRASP_WIDTH = self.get_parameter('min_grasp_width').value
        self.MAX_GRASP_PITCH_ROLL_DEG = self.get_parameter('max_grasp_pitch_roll_deg').value
        self.NMS_TRANSLATION_THRESH = self.get_parameter('nms_translation_thresh').value
        self.NMS_ANGLE_THRESH_DEG = self.get_parameter('nms_angle_thresh_deg').value
        self.POINTCLOUD_FROM_FILE = self.get_parameter('pointcloud_from_file').value
        self.POINTCLOUD_FILE = self.get_parameter('pointcloud_file').value
        self.MASK_RADIUS = self.get_parameter('mask_radius').value
        self.NO_RGB = self.get_parameter('no_rgb').value
        self.POINTCLOUD_TOPIC = self.get_parameter('pointcloud_topic').value
        self.VISUALISE = self.get_parameter('visualise').value
        self.MAX_GRIPPER_WIDTH = self.get_parameter('max_gripper_width').value
        self.GRIPPER_HEIGHT = self.get_parameter('gripper_height').value
        self.TOP_DOWN_GRASP = self.get_parameter('top_down_grasp').value
        self.GRASP_Z_OFFSET = self.get_parameter('grasp_z_offset').value
        self.GRASP_AXIS_OFFSET = self.get_parameter('grasp_axis_offset').value

        self.depth_pointcloud_: PointCloud2 = None

        if self.POINTCLOUD_FROM_FILE:
            from_file = np.load(self.POINTCLOUD_FILE)
            self.file_points = from_file['points']
            if not self.NO_RGB:
                self.file_colors = from_file['colors']

        cfgs = argparse.Namespace(
            checkpoint_path=checkpoint_path,
            max_gripper_width=self.MAX_GRIPPER_WIDTH,
            gripper_height=self.GRIPPER_HEIGHT,
            top_down_grasp=self.TOP_DOWN_GRASP,
            debug=True,
        )
        cfgs.max_gripper_width = max(0, min(0.1, cfgs.max_gripper_width))
        self.anygrasp = AnyGrasp(cfgs)
        self.anygrasp.load_net()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pointcloud_listener_ = self.create_subscription(PointCloud2, self.POINTCLOUD_TOPIC, self.pointcloud_callback_, qos)

        self.grasp_service_ = ActionServer(self, RequestGrasp, 'request_grasp', self.grasp_callback_)
        
        self.display_service_ = self.create_service(DisplayCloud, 'display_cloud', self.display_callback_)

        self.grasp_validity_client_ = ActionClient(self, ComputeGraspValidity, "compute_grasp_validity")

        self.get_logger().info("Started AnyGrasp Node")

    def display_grasps(gg, cloud, only_first=False, origin_position=[0,0,0], description=""):
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

    def display_pointcloud(points, colors=None, save=False, filename=None, origin_position=[0,0,0], description=""):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors) 
        if save:
            now = int(time.time())
            o3d.io.write_point_cloud(f"/ros2_ws/src/{filename}_{now}.pcd", pcd, write_ascii=True)
            np.savez(f"/ros2_ws/src/{filename}_{now}.npz", points=points, colors=colors)
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=origin_position)
        o3d.visualization.draw_geometries([pcd, origin_frame], window_name=description)

    def pointcloud_callback_(self, msg):
        self.depth_pointcloud_ = msg

    def display_callback_(self, request, response):
        # Convert PointCloud2 to numpy array
        self.get_logger().info("Displaying...")
        if self.depth_pointcloud_ is None:
            self.get_logger().info("Found no points")

        if self.NO_RGB:
            points, colors = self.fast_norgb_pc2_to_numpy(self.depth_pointcloud_)
        else:
            points, colors = self.fast_pc2_to_numpy(self.depth_pointcloud_)

        if points.shape[0] == 0:
            self.get_logger().info("No points found")
            return response

        # Open3D visualization
        AnyGraspNode.display_pointcloud(points, colors)

        return response

    def fast_norgb_pc2_to_numpy(self, msg):
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

    def fast_pc2_to_numpy(self, msg):
        # structured dtype matching your fields
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

    def generate_pose_(self, x, y, z, radius):
        if self.NO_RGB:
            if not self.POINTCLOUD_FROM_FILE:
                points, colors = self.fast_norgb_pc2_to_numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colors = np.zeros_like(points, dtype=np.float32)
            if self.VISUALISE:
                AnyGraspNode.display_pointcloud(points, save=True, filename="full_cloud", description="Full Point Cloud")
        else:
            if not self.POINTCLOUD_FROM_FILE:
                points, colors = self.fast_pc2_to_numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colors = self.file_colors
            if self.VISUALISE:
                AnyGraspNode.display_pointcloud(points, colors, save=True, filename="full_cloud", description="Full Point Cloud")

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
                AnyGraspNode.display_pointcloud(cropped_points, description="Cropped Point Cloud")
                AnyGraspNode.display_pointcloud(uncropped_points, description="Uncropped Point Cloud")
            else:
                AnyGraspNode.display_pointcloud(cropped_points, cropped_colors, description="Cropped Point Cloud")
                AnyGraspNode.display_pointcloud(uncropped_points, uncropped_colors, description="Uncropped Point Cloud")

        # set workspace to filter output grasps
        xmin, xmax = self.X_GRASP_MIN, self.X_GRASP_MAX
        ymin, ymax = self.Y_GRASP_MIN, self.Y_GRASP_MAX
        zmin, zmax = self.Z_GRASP_MIN, self.Z_GRASP_MAX
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        gg, cloud = self.anygrasp.get_grasp(
            cropped_points, cropped_colors, 
            lims=lims, 
            apply_object_mask=self.APPLY_OBJECT_MASK, 
            dense_grasp=self.DENSE_GRASP, 
            collision_detection=self.APPLY_COLLISIONS
            )

        if gg is None or len(gg) == 0:
            self.get_logger().error('No Grasp detected after collision detection!')
            return None
        
        if self.VISUALISE:
            AnyGraspNode.display_grasps(gg, cloud, origin_position=[x,y,z], description="All Grasps")

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

            # if abs(yaw) > 90:
            #     exclude_grasps.append(ind)

        gg.remove(exclude_grasps)

        if len(gg) == 0:
            self.get_logger().error('No Grasps obtained after orientation filtering performed')
            return

        # perform non-maximum suppression
        gg = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        ).sort_by_score()

        # visualization
        if self.VISUALISE:
            AnyGraspNode.display_grasps(gg, cloud, origin_position=[x,y,z], description="Filtered Grasps")
            AnyGraspNode.display_grasps(gg, cloud, only_first=True, origin_position=[x,y,z], description="Highest Grasp Score")

        return gg, cloud

    async def grasp_callback_(self, goal_handle):
        request = goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()

        if not self.POINTCLOUD_FROM_FILE and self.depth_pointcloud_ is None:
            self.get_logger().error("PointCloud Not Available")
            response.success = False
            response.message = "Point cloud not available"
            goal_handle.abort()
            return response

        gg, cloud = self.generate_pose_(request.x, request.y, request.z, self.MASK_RADIUS)
        if gg is None or len(gg) == 0:
            self.get_logger().error("Grasp Failed")
            response.success = False
            response.message = "Unable to identify any grasps"
            goal_handle.abort()
            return response
        
        feedback.progress = f"Identified {len(gg)} grasps. Evaluating efficiency..."
        goal_handle.publish_feedback(feedback)

        min_score = math.inf
        max_score = -math.inf

        grasp_scores = []

        for ind, grasp in enumerate(gg):
            goal = ComputeGraspValidity.Goal()
            goal.grasp_pose = self.map_grasp(grasp)
            goal.goal_x = request.goal_x
            goal.goal_y = request.goal_y
            goal.goal_z = request.goal_z
            goal.target_id = request.id
            self.get_logger().info(f"Candidate Grasp {ind + 1}/{len(gg)}: " + str(goal.grasp_pose))

            self.grasp_validity_client_.wait_for_server()
            send_goal_future = self.grasp_validity_client_.send_goal_async(goal)

            while not send_goal_future.done():
                rclpy.spin_once(self)
                time.sleep(0.01)

            goal_handle_inner = send_goal_future.result()

            if not goal_handle_inner.accepted:
                self.get_logger().warn("Goal Rejected")
                continue

            result_future = goal_handle_inner.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            if not result.is_valid:
                self.get_logger().info("Not a valid pose")
                grasp_scores.append(math.inf)
            else:
                self.get_logger().info("Valid pose with score " + str(result.score))            
                grasp_scores.append(result.score)

                if result.score > max_score:
                    max_score = result.score
                if result.score < min_score:
                    min_score = result.score

        trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        cloud.transform(trans_mat)
        grippers = gg.to_open3d_geometry_list()
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1,      # length of the axes
            origin=[0,0,0]
        )
        for ind, g in enumerate(grippers):
            g.transform(trans_mat)
            if math.isinf(grasp_scores[ind]):
                color = np.array([[0.0], [0.0], [0.0]], dtype=np.float64)
                g.paint_uniform_color(color)
            else:
                color = np.array(cm.RdYlGn_r((grasp_scores[ind] - min_score)/(max_score - min_score + 0.0001))[:3], dtype=np.float64)
                g.paint_uniform_color(color)
        o3d.visualization.draw_geometries([*grippers, cloud, origin_frame], window_name="Scored Grasps")

        num_valid_grasps = sum([not math.isinf(score) for score in grasp_scores ])
        feedback.progress = f"Evaluated efficiency. {num_valid_grasps} valid grasps found."
        goal_handle.publish_feedback(feedback)

        if num_valid_grasps:
            self.get_logger().info("Found the best grasp")

            best_grasp_score = min(grasp_scores)
            best_grasp_ind = grasp_scores.index(best_grasp_score)
            best_grasp = gg[best_grasp_ind]

            final_grasp_group = GraspGroup()
            final_grasp_group.add(best_grasp)
            AnyGraspNode.display_grasps(final_grasp_group, cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

            response.grasp_pose = self.map_grasp(best_grasp)
            self.get_logger().info("--> " + str(response.grasp_pose))
            response.success = True        
            goal_handle.succeed()
        else:
            self.get_logger().info("No valid grasps found")
            response.success = False
            goal_handle.abort()
        return response
    
    def map_grasp(self, grasp):
        grasp_rotation = Rotation.from_matrix(grasp.rotation_matrix)
        offset_rotation = Rotation.from_euler('y', 90, degrees=True)
        final_rotation = grasp_rotation * offset_rotation
        final_quaternion = final_rotation.as_quat()

        # local axes:
            # z points in the direction of grasp attack
            # y is perpendicular to z in the horizontal plane
            # x points vertical

        offset_translation = np.array([0, 0, -self.GRASP_AXIS_OFFSET])
        grasp.translation[2] += self.GRASP_Z_OFFSET
        final_translation = grasp.translation + final_rotation.as_matrix() @ offset_translation

        pose = Pose()
        pose.position.x = final_translation[0]
        pose.position.y = final_translation[1]
        pose.position.z = final_translation[2]
        pose.orientation.x = final_quaternion[0]
        pose.orientation.y = final_quaternion[1]
        pose.orientation.z = final_quaternion[2]
        pose.orientation.w = final_quaternion[3]

        return pose

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