import os
import time
import argparse
import numpy as np
import open3d as o3d

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_prefix
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from hts_msgs.srv import RequestGrasp, DisplayCloud
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Pose

pkg_prefix = get_package_prefix("hts_anygrasp")
lib_path = os.path.join(pkg_prefix, "lib", "hts_anygrasp")
os.environ["LD_LIBRARY_PATH"] = (lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", ""))
checkpoint_path = os.path.join(pkg_prefix, "share/hts_anygrasp/checkpoint_detection.tar")

from graspnetAPI import GraspGroup
from gsnet import AnyGrasp

qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT

from_file = np.load("/ros2_ws/src/full_cloud_1771641840.npz")
from_file_points = from_file['points']

class AnyGraspNode(Node):
    Z_COORDS_MIN = 0.001
    Z_COORDS_MAX = 0.5
    X_GRASP_MIN = -1.0
    X_GRASP_MAX = 1.0
    Y_GRASP_MIN = -1.0
    Y_GRASP_MAX = 1.0
    Z_GRASP_MIN = 0.03
    Z_GRASP_MAX = 0.5

    APPLY_OBJECT_MASK = True
    APPLY_COLLISIONS = True
    DENSE_GRASP = True

    MIN_GRASP_WIDTH = 0.01
    MAX_GRASP_PITCH_ROLL_DEG = 5

    NMS_TRANSLATION_THRESH = 0.005
    NMS_ANGLE_THRESH_DEG = 5

    MASK_RADIUS = 0.05

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
            AnyGraspNode.display_pointcloud(points, save=True, filename="full_cloud")
            points = from_file_points
            colors = np.zeros_like(points, dtype=np.float32)
        else:
            points, colors = self.fast_pc2_to_numpy(self.depth_pointcloud_)
            AnyGraspNode.display_pointcloud(points, colors, save=True, filename="full_cloud")


        # filter according to z
        z_coords = points[:, 2]
        y_coords = points[:, 1]
        x_coords = points[:, 0]
        mask = (z_coords > AnyGraspNode.Z_COORDS_MIN) & (z_coords < AnyGraspNode.Z_COORDS_MAX) & ((x - x_coords)**2 + (y - y_coords)**2 < radius**2)
        
        cropped_points = points[mask].astype(np.float32)
        cropped_colors = colors[mask].astype(np.float32)
        uncropped_points = points[~mask].astype(np.float32)
        uncropped_colors = colors[~mask].astype(np.float32)

        if not cropped_points.shape[0]:
            self.get_logger().info("Cropped pointcloud contains no points")
            return None

        # show cropped and uncropped pointclouds
        if self.NORGB:
            AnyGraspNode.display_pointcloud(cropped_points)
            AnyGraspNode.display_pointcloud(uncropped_points)
        else:
            AnyGraspNode.display_pointcloud(cropped_points, cropped_colors)
            AnyGraspNode.display_pointcloud(uncropped_points, uncropped_colors)

        # set workspace to filter output grasps
        xmin, xmax = AnyGraspNode.X_GRASP_MIN, AnyGraspNode.X_GRASP_MAX
        ymin, ymax = AnyGraspNode.Y_GRASP_MIN, AnyGraspNode.Y_GRASP_MAX
        zmin, zmax = AnyGraspNode.Z_GRASP_MIN, AnyGraspNode.Z_GRASP_MAX
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        gg, cloud = self.anygrasp.get_grasp(
            cropped_points, cropped_colors, 
            lims=lims, 
            apply_object_mask=AnyGraspNode.APPLY_OBJECT_MASK, 
            dense_grasp=AnyGraspNode.DENSE_GRASP, 
            collision_detection=AnyGraspNode.APPLY_COLLISIONS
            )

        if gg is None or len(gg) == 0:
            self.get_logger().info('No Grasp detected after collision detection!')
            return None
        
        exclude_grasps = []
        for ind, grasp in enumerate(gg):
            # exclude grasps by width
            if grasp.width < AnyGraspNode.MIN_GRASP_WIDTH:
                exclude_grasps.append(ind)
                continue

            # exclude grasps by orientation
            roll, pitch, yaw = Rotation.from_matrix(grasp.rotation_matrix).as_euler('xyz', degrees=True)
            if abs(pitch) > AnyGraspNode.MAX_GRASP_PITCH_ROLL_DEG and abs(pitch - 180) > AnyGraspNode.MAX_GRASP_PITCH_ROLL_DEG:
                exclude_grasps.append(ind)
                continue
            if abs(roll) > AnyGraspNode.MAX_GRASP_PITCH_ROLL_DEG and abs(roll - 180) > AnyGraspNode.MAX_GRASP_PITCH_ROLL_DEG:
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
            translation_thresh = AnyGraspNode.NMS_TRANSLATION_THRESH,
            rotation_thresh = AnyGraspNode.NMS_ANGLE_THRESH_DEG / 180 * np.pi
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

        offset_translation = np.array([0, 0, -0.14])
        grasp.translation[2] += 0.02
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