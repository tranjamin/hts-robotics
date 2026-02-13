import rclpy
from rclpy.node import Node
from hts_msgs.srv import RequestGrasp
from sensor_msgs.msg import PointCloud2
import os

import argparse
import torch
import numpy as np
import open3d as o3d
from PIL import Image
from ament_index_python.packages import get_package_prefix
from graspnetAPI import GraspGroup

pkg_prefix = get_package_prefix("hts_anygrasp")
lib_path = os.path.join(pkg_prefix, "lib", "hts_anygrasp")

os.environ["LD_LIBRARY_PATH"] = (
    lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")
)

from gsnet import AnyGrasp

class AnyGraspNode(Node):
    def __init__(self):
        super().__init__('hts_anygrasp')

        self.depth_listener_ = self.create_subscription(PointCloud2, "/camera/camera/depth/color/points", self.depth_callback_, 1)
        self.grasp_service_ = self.create_service(RequestGrasp, 'request_grasp', self.grasp_callback_)

    def depth_callback_(self, msg):
        self.depth_pointcloud_ = msg

    def grasp_callback_(self, request, response):
        self.get_logger().info("Requested pose for object %d" % (request.id,))

def main(args=None):
    print("Starting AnyGrasp Node")
    rclpy.init(args=args)
    node = AnyGraspNode()
    print("Started AnyGrasp Node")
    rclpy.spin(node)
    rclpy.shutdown()
    print("Stopped AnyGrasp Node")

if __name__ == '__main__':
    main()