import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from hts_msgs.srv import RequestGrasp, DisplayCloud
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import os

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

        # self.pointcloud_listener_ = self.create_subscription(PointCloud2, "/camera_sim/points", self.pointcloud_callback_, 1)
        self.pointcloud_listener_ = self.create_subscription(PointCloud2, "/octomap_point_cloud_centers", self.pointcloud_callback_, qos)
        self.depth_listener_ = self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_callback_, 1)
        self.rgb_listener_ = self.create_subscription(Image, "/camera/camera/color/image_raw", self.rgb_callback_, 1)
        self.grasp_service_ = self.create_service(RequestGrasp, 'request_grasp', self.grasp_callback_)
        self.display_service_ = self.create_service(DisplayCloud, 'display_cloud', self.display_callback_)

        self.get_logger().info("Started AnyGrasp Node")

    def pointcloud_callback_(self, msg):
        self.depth_pointcloud_ = msg

    def display_callback_(self, request, response):
        # Convert PointCloud2 to numpy array
        self.get_logger().info("Displaying...")
        if self.depth_pointcloud_ is None:
            self.get_logger().info("Found no points")

        points, colors = self.fast_norgb_pc2_to_numpy(self.depth_pointcloud_)

        if points.shape[0] == 0:
            self.get_logger().info("No points found")
            return response

        # Open3D visualization
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd])

        self.get_logger().info("done")

        return response

    def depth_callback_(self, msg):
        self.depth_image_ = msg

    def rgb_callback_(self, msg):
        self.rgb_image_ = msg

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

    def generate_pose_(self, x, y, z, xrange, yrange, zrange):
        points, colors = self.fast_norgb_pc2_to_numpy(self.depth_pointcloud_)

        # filter according to z
        z_coords = points[:, 2]
        y_coords = points[:, 1]
        x_coords = points[:, 0]
        mask = (z_coords > 0.03) & (z_coords < 0.2) & (x_coords > x - xrange) & (x_coords < x + xrange) & (y_coords > y - yrange) & (y_coords < y + yrange)
        points = points[mask].astype(np.float32)
        colors = colors[mask].astype(np.float32)

        self.get_logger().info("Finished converting pointcloud")

        np.savez("/ros2_ws/src/cloud.npz", points=points, colors=colors)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors) 
        o3d.io.write_point_cloud("/ros2_ws/src/cloud.pcd", pcd, write_ascii=True)

        # depth_img = self.bridge.imgmsg_to_cv2(self.depth_image_, desired_encoding='32FC1')
        # rgb_img = self.bridge.imgmsg_to_cv2(self.rgb_image_, desired_encoding='bgr8')

        # if self.cfgs.debug:
        #     cv2.imwrite("/ros2_ws/src/Depth Image.png", depth_img / depth_img.max() * 255)
        #     cv2.imwrite("/ros2_ws/src/RGB Image.png", rgb_img)

        # set workspace to filter output grasps
        xmin, xmax = -2, 2
        ymin, ymax = -2, 2
        zmin, zmax = 0.03, 0.2
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        gg, cloud = self.anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)

        if gg is None or len(gg) == 0:
            self.get_logger().info('No Grasp detected after collision detection!')
            return None

        gg = gg.nms().sort_by_score()
        gg_pick = gg[0:20]
        self.get_logger().info(str(gg_pick.scores))
        self.get_logger().info('grasp score:' + str(gg_pick[0].score))

        # visualization
        if self.cfgs.debug:
            trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
            cloud.transform(trans_mat)
            grippers = gg.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(trans_mat)
            o3d.visualization.draw_geometries([*grippers, cloud])
            o3d.visualization.draw_geometries([grippers[0], cloud])

        return gg_pick[0]

    def grasp_callback_(self, request, response):
        if self.depth_pointcloud_ is None:
            self.get_logger().info("PointCloud Not Available")
            response.success = False
            return response

        self.get_logger().info("Requested pose for object %d" % (request.id,))

        grasp = self.generate_pose_(request.x, request.y, request.z, 0.3, 0.3, 0.3)
        self.get_logger().info(str(grasp))

        if grasp is None:
            self.get_logger().info("Grasp Failed")
            response.success = False
            return response

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