import os
import time
import argparse
import numpy as np
import open3d as o3d

from ament_index_python.packages import get_package_prefix
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from hts_msgs.srv import RequestGrasp, DisplayCloud
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
        self.declare_parameter('pointcloud_topic', '/octomap_point_cloud_centres')
        self.declare_parameter('visualise', True)
        self.declare_parameter('max_gripper_width', 0.1)
        self.declare_parameter('gripper_height', 0.03)
        self.declare_parameter('top_down_grasp', False)
        self.declare_parameter('grasp_z_offset', 0.02)
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
        self.grasp_service_ = self.create_service(RequestGrasp, 'request_grasp', self.grasp_callback_)
        self.display_service_ = self.create_service(DisplayCloud, 'display_cloud', self.display_callback_)

        self.get_logger().info("Started AnyGrasp Node")

    def display_grasps(gg, cloud, only_first=False, origin_position=[0,0,0]):
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
            o3d.visualization.draw_geometries([*grippers, cloud, origin_frame])
        else:
            o3d.visualization.draw_geometries([grippers[0], cloud, origin_frame])

    def display_pointcloud(points, colors=None, save=False, filename=None, origin_position=[0,0,0]):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors) 
        if save:
            now = int(time.time())
            o3d.io.write_point_cloud(f"/ros2_ws/src/{filename}_{now}.pcd", pcd, write_ascii=True)
            np.savez(f"/ros2_ws/src/{filename}_{now}.npz", points=points, colors=colors)
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=origin_position)
        o3d.visualization.draw_geometries([pcd, origin_frame])

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
                AnyGraspNode.display_pointcloud(points, save=True, filename="full_cloud")
        else:
            if not self.POINTCLOUD_FROM_FILE:
                points, colors = self.fast_pc2_to_numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colors = self.file_colors
            if self.VISUALISE:
                AnyGraspNode.display_pointcloud(points, colors, save=True, filename="full_cloud")

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
            self.get_logger().info("Cropped pointcloud contains no points")
            return None

        # show cropped and uncropped pointclouds
        if self.VISUALISE:
            if self.NO_RGB:
                AnyGraspNode.display_pointcloud(cropped_points)
                AnyGraspNode.display_pointcloud(uncropped_points)
            else:
                AnyGraspNode.display_pointcloud(cropped_points, cropped_colors)
                AnyGraspNode.display_pointcloud(uncropped_points, uncropped_colors)

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
            self.get_logger().info('No Grasp detected after collision detection!')
            return None
        
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

            if abs(yaw) > 90:
                exclude_grasps.append(ind)

        gg.remove(exclude_grasps)

        if len(gg) == 0:
            self.get_logger().info('No Grasps obtained after orientation filtering performed')
            return

        # perform non-maximum suppression
        gg = gg.nms(
            translation_thresh = self.NMS_TRANSLATION_THRESH,
            rotation_thresh = self.NMS_ANGLE_THRESH_DEG / 180 * np.pi
        ).sort_by_score()

        # visualization
        if self.VISUALISE:
            AnyGraspNode.display_grasps(gg, cloud, origin_position=[x,y,z])
            AnyGraspNode.display_grasps(gg, cloud, only_first=True, origin_position=[x,y,z])

        self.get_logger().info(str(gg.scores))
        self.get_logger().info('grasp score:' + str(gg[0].score))            
        return gg[0]

    def grasp_callback_(self, request, response):
        if not self.POINTCLOUD_FROM_FILE and self.depth_pointcloud_ is None:
            self.get_logger().info("PointCloud Not Available")
            response.success = False
            return response

        self.get_logger().info("Requested pose for object %d" % (request.id,))
        self.get_logger().info(f"Pose is centred at {request.x}, {request.y}, {request.z}")

        grasp = self.generate_pose_(request.x, request.y, request.z, self.MASK_RADIUS)
        self.get_logger().info(str(grasp))

        if grasp is None:
            self.get_logger().info("Grasp Failed")
            response.success = False
            return response

        self.get_logger().info("Identified grasp is " + str(grasp))
        self.get_logger().info("Pose orientation is " + str(Rotation.from_matrix(grasp.rotation_matrix).as_euler('xyz', degrees=True)))

        grasp_rotation = Rotation.from_matrix(grasp.rotation_matrix)
        offset_rotation = Rotation.from_euler('y', 90, degrees=True)
        final_rotation = grasp_rotation * offset_rotation
        final_quaternion = final_rotation.as_quat()
        print(final_quaternion)

        # local axes:
            # z points in the direction of grasp attack
            # y is perpendicular to z in the horizontal plane
            # x points vertical

        offset_translation = np.array([0, 0, -self.GRASP_AXIS_OFFSET])
        grasp.translation[2] += self.GRASP_Z_OFFSET
        final_translation = grasp.translation + final_rotation.as_matrix() @ offset_translation
        print(final_translation)

        pose = Pose()
        pose.position.x = final_translation[0]
        pose.position.y = final_translation[1]
        pose.position.z = final_translation[2]
        pose.orientation.x = final_quaternion[0]
        pose.orientation.y = final_quaternion[1]
        pose.orientation.z = final_quaternion[2]
        pose.orientation.w = final_quaternion[3]

        response.grasp_pose = pose
        response.success = True
        return response
        
def main():
    rclpy.init(args=None)

    node = AnyGraspNode()

    rclpy.spin(node)
    rclpy.shutdown()
    print("Stopped AnyGrasp Node")

if __name__ == '__main__':
    main()