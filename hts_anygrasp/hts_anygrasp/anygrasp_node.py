import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from hts_msgs.srv import RequestGrasp, DisplayCloud
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import os
import time

import argparse
import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_prefix
from graspnetAPI import GraspGroup
from scipy.spatial.transform import Rotation

pkg_prefix = get_package_prefix("hts_anygrasp")
lib_path = os.path.join(pkg_prefix, "lib", "hts_anygrasp")
os.environ["LD_LIBRARY_PATH"] = (lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", ""))
checkpoint_path = os.path.join(pkg_prefix, "share/hts_anygrasp/checkpoint_detection.tar")

qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT

from gsnet import AnyGrasp

class AnyGraspNode(Node):
    def __init__(self, args):
        super().__init__('hts_anygrasp')
        self.depth_pointcloud_: PointCloud2 = None
        self.depth_image_: Image = None
        self.rgb_image_: Image = None
        
        self.anygrasp = AnyGrasp(args)
        self.anygrasp.load_net()

        self.cfgs = args
        self.bridge = CvBridge()

        self.NORGB = True
        # self.pointcloud_listener_ = self.create_subscription(PointCloud2, "/camera_sim/points", self.pointcloud_callback_, 1)
        self.pointcloud_listener_ = self.create_subscription(PointCloud2, "/octomap_point_cloud_centers", self.pointcloud_callback_, qos)
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

    def display_pointcloud(points, colors, save=False, filename=None, origin_position=[0,0,0]):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
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

        if self.NORGB:
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
        # structured dtype matching your fields
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('pad', np.float32),   # offset 12–15 unused
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
            ('pad', np.float32),   # offset 12–15 unused
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
        if self.NORGB:
            points, colors = self.fast_norgb_pc2_to_numpy(self.depth_pointcloud_)
        else:
            points, colors = self.fast_pc2_to_numpy(self.depth_pointcloud_)

        AnyGraspNode.display_pointcloud(points, colors, save=True, filename="full_cloud")

        # filter according to z
        z_coords = points[:, 2]
        y_coords = points[:, 1]
        x_coords = points[:, 0]
        mask = (z_coords > 0.001) & (z_coords < 0.3) & ((x - x_coords)**2 + (y - y_coords)**2 < radius**2)
        
        cropped_points = points[mask].astype(np.float32)
        cropped_colors = colors[mask].astype(np.float32)
        uncropped_points = points[~mask].astype(np.float32)
        uncropped_colors = colors[~mask].astype(np.float32)

        if not cropped_points.shape[0]:
            self.get_logger().info("Cropped pointcloud contains no points")
            return None

        # show cropped and uncropped pointclouds
        AnyGraspNode.display_pointcloud(cropped_points, cropped_colors)
        AnyGraspNode.display_pointcloud(uncropped_points, uncropped_colors)

        # set workspace to filter output grasps
        xmin, xmax = -1, 1
        ymin, ymax = -1, 1
        zmin, zmax = 0.03, 0.5
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        gg, cloud = self.anygrasp.get_grasp(
            cropped_points, cropped_colors, 
            lims=lims, 
            apply_object_mask=True, 
            dense_grasp=True, 
            collision_detection=True
            )

        if gg is None or len(gg) == 0:
            self.get_logger().info('No Grasp detected after collision detection!')
            return None
        
        exclude_grasps = []
        for ind, grasp in enumerate(gg):
            # exclude grasps by width
            if grasp.width < 0.01:
                exclude_grasps.append(ind)
                continue

            # exclude grasps by orientation
            roll, pitch, yaw = Rotation.from_matrix(grasp.rotation_matrix).as_euler('xyz', degrees=True)
            if abs(pitch) > 5 and abs(pitch - 180) > 5:
                exclude_grasps.append(ind)
                continue
            if abs(roll) > 5 and abs(roll - 180) > 5:
                exclude_grasps.append(ind)
                continue

        gg.remove(exclude_grasps)

        if len(gg) == 0:
            self.get_logger().info('No Grasps obtained after orientation filtering performed')
            return

        # perform non-maximum suppression
        gg = gg.nms(
            translation_thresh = 0.005,
            rotation_thresh = 5 / 180 * np.pi
        ).sort_by_score()

        # visualization
        if self.cfgs.debug:
            AnyGraspNode.display_grasps(gg, cloud, origin_position=[x,y,z])
            AnyGraspNode.display_grasps(gg, cloud, only_first=True, origin_position=[x,y,z])

        self.get_logger().info(str(gg.scores))
        self.get_logger().info('grasp score:' + str(gg[0].score))            
        return gg[0]

    def grasp_callback_(self, request, response):
        if self.depth_pointcloud_ is None:
            self.get_logger().info("PointCloud Not Available")
            response.success = False
            return response

        self.get_logger().info("Requested pose for object %d" % (request.id,))
        self.get_logger().info(f"Pose is centred at {request.x}, {request.y}, {request.z}")

        grasp = self.generate_pose_(request.x, request.y, request.z, 0.05)
        self.get_logger().info(str(grasp))

        if grasp is None:
            self.get_logger().info("Grasp Failed")
            response.success = False
            return response

        self.get_logger().info("Identified grasp is " + str(grasp))
        self.get_logger().info("Pose orientation is " + str(Rotation.from_matrix(grasp.rotation_matrix).as_euler('xyz', degrees=True)))

        quaternion = Rotation.from_matrix(grasp.rotation_matrix).as_quat()

        pose = Pose()
        pose.position.x = grasp.translation[0]
        pose.position.y = grasp.translation[1]
        pose.position.z = grasp.translation[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        response.grasp_pose = pose
        response.success = True
        return response
        
def main():
    rclpy.init(args=None)

    cfgs = argparse.Namespace(
        checkpoint_path=checkpoint_path,
        max_gripper_width=0.1,
        gripper_height=0.03,
        top_down_grasp=False,
        debug=True,
    )
    cfgs.max_gripper_width = max(0, min(0.1, cfgs.max_gripper_width))

    node = AnyGraspNode(cfgs)

    rclpy.spin(node)
    rclpy.shutdown()
    print("Stopped AnyGrasp Node")

if __name__ == '__main__':
    main()

def filter_by_orientation(gg):
    exclude_grasps = []
    for ind, grasp in enumerate(gg):
        # exclude grasps by width
        if grasp.width < 0.01:
            exclude_grasps.append(ind)
            continue
        # exclude grasps by orientation
        roll, pitch, yaw = Rotation.from_matrix(grasp.rotation_matrix).as_euler('xyz', degrees=True)
        if (abs(pitch) > 20 and abs(pitch - 180) > 20) or (abs(roll) > 20 and abs(roll - 180) > 20):
            exclude_grasps.append(ind)
            continue
    gg.remove(exclude_grasps)