from __future__ import annotations
import os
import time
import argparse
import math
import matplotlib.cm as cm
import threading
import numpy as np
import open3d as o3d
import time
import typing
import copy
import scipy.stats
import functools
import random
import matplotlib.pyplot as plt

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
from graspnetAPI import GraspGroup
from gsnet import AnyGrasp

class HTSGrasp():
    def __init__(self):
        self.grasp_score: float = 0.0
        self.path_score: float = 0.0

        self.grasp_object: GraspNetGrasp = None
        self.pose: Pose = None
        self._t0 = 0.0

        self.result_message = ""
        self.result_err_code = 0
        self.result_err_source = ""
        self.result_err_message = ""
        self.result_timings = []
        self._is_valid = False
        self.path_score = 0.0
        self._evaluated = False
        self.time = 0.0
    
    def set_pose(self, pose: Pose):
        self.pose = pose

    def get_pose(self) -> Pose:
        return self.pose
    
    def set_grasp_object(self, grasp: GraspNetGrasp):
        self.grasp_object = grasp
        self.grasp_score = grasp.score
    
    def get_grasp_object(self) -> GraspNetGrasp:
        return self.grasp_object

    def single_grasp_group(self) -> GraspGroup:
        temp_grasp_group = GraspGroup()
        temp_grasp_group.add(self.get_grasp_object())
        return temp_grasp_group
        
    def validity_request_goal(self, request: ComputeGraspValidity.Request) -> ComputeGraspValidity.Goal:
        goal = ComputeGraspValidity.Goal()
        goal.grasp_pose = self.get_pose()
        goal.goal_x = request.goal_x
        goal.goal_y = request.goal_y

        # out goal z is expressed as an offset
        goal.goal_z = request.goal_z + goal.grasp_pose.position.z
        goal.target_id = request.id

        return goal

    def process_result(self, result: ComputeGraspValidity.Result):
        self.result_message = result.message
        self.result_err_code = result.err_code
        self.result_err_source = result.err_source
        self.result_err_message = result.err_message
        self.result_timings = [result.pickup_ik_time, result.pickup_plan_time, result.pickup_refine_time, result.move_ik_time, result.move_plan_time, result.move_refine_time]
        self._is_valid = result.is_valid
        self.path_score = result.score if result.is_valid else math.inf
        self._evaluated = True
    
    def is_valid(self) -> bool:
        return self._is_valid
    
    def pickup_failed(self) -> bool:
        return self.result_err_source == "pickup"

    def move_failed(self) -> bool:
        return self.result_err_source == "move"
    
    def evaluated(self) -> bool:
        return self._evaluated

    def save_grasp_message(self, folder: str):
        with open(f"{folder}/grasp_messages.txt", "a") as f:
            f.write(f"Message {self.result_message} | Err Code {self.result_err_code} | Err Source {self.result_err_source} | Err Msg {self.result_err_message}\r\n")
    
    def start_timer(self):
        self._t0 = time.time()
    
    def end_timer(self):
        self.time = time.time() - self._t0

    def save_grasp_validity(self, folder: str, include_header: bool=False):
        with open(f"{folder}/grasp_validities.txt", "a") as f:
            if include_header:
                f.write(f"planning_time,planing_score,grasp_score,pickup_ik_t,pickup_plan_t,pickup_refine_t,move_ik_t,move_plan_t,move_refine_t\r\n") 
            f.write(f"{self.time},{self.path_score},{self.grasp_score},{str(self.result_timings)[1:-1]}\r\n")
    
    def save_grasp_info(self, folder: str):
        pose = self.get_pose()
        with open(f"{folder}/grasps.txt", "a") as f:
            f.write(f"{pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.x} {pose.orientation.y} {pose.orientation.z} {pose.orientation.w}\r\n")

class HTSGraspGroup():
    def __init__(self, grasp_list=None):
        self._grasps: list[HTSGrasp] = [] if grasp_list is None else grasp_list
    
    def append(self, hts_grasp: HTSGrasp):
        self._grasps.append(hts_grasp)
        
    def concat(self, hts_grasp_group: HTSGraspGroup):
        self._grasps.append(hts_grasp_group._grasps)
        
    def __len__(self):
        return len(self._grasps)
    
    def num_valid(self) -> int:
        return len([grasp for grasp in self._grasps if grasp.is_valid()])

    def save_metrics(self, folder: str):
        valid_grasps = [grasp for grasp in self._grasps if grasp.is_valid()]
        valid_scores = [grasp.path_score for grasp in valid_grasps]
        total_time = sum([grasp.time for grasp in self._grasps])

        with open(f"{folder}/grasp_metrics.txt", "a") as f:
            f.write(f"Total Grasps: {len(self)}\r\n")
            f.write(f"Valid Grasps: {len(valid_grasps)}\r\n")
            f.write(f"Percent Valid Grasps: {(len(valid_grasps) / len(self)) if len(self) else '-'}\r\n")
            f.write(f"Total Planning Time: {total_time}\r\n")
            f.write(f"Average Planning Time: {(total_time / len(self)) if len(self) else '-'}\r\n")
            if len(valid_grasps):
                f.write(f"Average Planning Distance: {sum(valid_scores) / len(valid_grasps) if len(valid_grasps) else '-'}\r\n")
            f.write(f"Shortest Planning Distance: {min(valid_scores) if len(valid_grasps) else '-'}\r\n")

    def best_grasp(self) -> HTSGrasp:
        self._grasps.sort(key=lambda x: x.path_score)
        return self._grasps[0]
    
    def valid_grasp_group(self) -> HTSGraspGroup:
        return HTSGraspGroup([grasp for grasp in self._grasps if grasp.is_valid()])
    
    def filter_grasp_group(self, filter) -> HTSGraspGroup:
        return HTSGraspGroup([grasp for grasp in self._grasps if filter(grasp)])

    def visualise(self, cloud, origin_position=[0,0,0], description="HTS Grasp Group"):
        valid_grasps = [grasp for grasp in self._grasps if grasp.is_valid()]

        min_score = min([grasp.path_score for grasp in valid_grasps]) if len(valid_grasps) else 0
        max_score = max([grasp.path_score for grasp in valid_grasps]) if len(valid_grasps) else 0
        
        gg = GraspGroup()

        for hts_grasp in self._grasps:
            gg.add(hts_grasp.get_grasp_object())
        
        grippers = gg.to_open3d_geometry_list()

        trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        for ind, g in enumerate(grippers):
            g.transform(trans_mat)
            if not self._grasps[ind].is_valid():
                color = np.array([[0.0], [0.0], [0.0]], dtype=np.float64)
                g.paint_uniform_color(color)
            elif min_score != max_score:
                color = np.array(cm.RdYlGn_r((self._grasps[ind].path_score - min_score)/(max_score - min_score))[:3], dtype=np.float64)
                g.paint_uniform_color(color)

        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1,      # length of the axes
            origin=origin_position
        )

        cloud.transform(trans_mat)

        o3d.visualization.draw_geometries([*grippers, cloud, origin_frame], window_name=description)

class SymmetryGroup():
    def __init__(self, cloud: o3d.cuda.pybind.geometry.PointCloud, gg, logger):
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
    
# construct a problem space
class ProblemSpace():
    lambda1: float = 1.0
    lambda2: float = 30.0
    
    weibull_k: float = 0.7
    weibull_gamma: float = 0.5
    weibull_base: float = 1.3

    sigma_theta: float = 0.4
    sigma_z: float = 0.05
    
    select_greedy_eps: float = 0.7

    certainty_scaler_range: float = 1e-3
    prediction_z_range: float = 0.05
    prediction_th_range: float = math.pi/4
    
    total_max_time = 30.0
    
    def __init__(self, node: AnyGraspNode, hts_gg: HTSGraspGroup):
        self.grasps: HTSGraspGroup = hts_gg # all grasps
        self.node: AnyGraspNode = node # node
        
        self.points: list[ProblemPoints] = [ProblemPoints(g) for g in hts_gg._grasps] # points
        self.t0 = 0.0

        self.max_path_score = 0.0

        zs = np.array([p.z for p in self.points])
        thetas = np.array([p.theta for p in self.points])
        np.save("/ros2_ws/src/z", zs)
        np.save("/ros2_ws/src/th", thetas)

        self.plot_grasps()
    
    def start_timer(self):
        self.t0 = time.time()
    
    def select_next(self):
        self.node.get_logger().info("Select Next")
        # highest uncertainty
        uncertanties = [1 - x.get_certainty() for x in self.points]
        
        costs = [x.cost(self.max_path_score) for x in self.points]
        if min(costs) < 0:
            costs = [x - min(costs) for x in costs]
        weighted_cost = [costs[i]*uncertanties[i] for i in range(len(costs))]

        # if everything is uncertain, then select greedily
        if sum(weighted_cost) != 0.0 and (min(uncertanties) == 1.0 or random.random() < self.select_greedy_eps):
            if random.random() < 0.5:
                # print(f"Trying to sample greedily")
                idx = random.choices(range(len(self.points)), weights=weighted_cost)[0]
                self.node.get_logger().info(f"Selecting probabilistic greedily {idx}")
                return idx, self.points[idx]
            else:
                # print(f"Trying to sample greedily")
                idx = weighted_cost.index(max(weighted_cost))
                self.node.get_logger().info(f"Selecting greedily {idx}")
                return idx, self.points[idx]
        elif sum(uncertanties) != 0.0:
            if random.random() < 0.5:
                # print(f"Trying to sample uncertainty")
                idx = random.choices(range(len(self.points)), weights=uncertanties)[0]
                self.node.get_logger().info(f"Selecting greedily based on uncertainty {idx}")
                return idx, self.points[idx]
            else:
                # print(f"Trying to sample greedily")
                idx = uncertanties.index(max(uncertanties))
                self.node.get_logger().info(f"Selecting based on uncertainty {idx}")
                return idx, self.points[idx]
        else:
            self.node.get_logger().info("Everything is 100%% certain now")
            return 0, None
        
    def plot_grasps(self, folder=None):
        zs = np.array([p.z for p in self.points])
        thetas = np.array([p.theta for p in self.points])

        x = zs*np.cos(thetas)
        y = zs*np.sin(thetas)

        path_scores = [min(self.max_path_score, x.path_score) for x in self.points]
        grasp_scores = [x.grasp_score for x in self.points]
        
        uncertanties = [1 - x.get_certainty() for x in self.points]
        costs = [x.cost(self.max_path_score) for x in self.points]
        if min(costs) < 0:
            costs = [x - min(costs) for x in costs]
        weighted_cost = [costs[i]*uncertanties[i] for i in range(len(costs))]

        evaluated = [x.evaluated for x in self.points]
        distance = [ProblemPoints._distance_scaler(p.z, p.theta) for p in self.points]

        fig, axs = plt.subplots(2, 3, figsize=(24,12))
        splt1 = axs[0, 0].scatter(x, y, c=uncertanties, vmin=0.0, vmax=1.0)
        splt2 = axs[0, 1].scatter(x, y, c=costs, vmin=0.0)
        splt3 = axs[0, 2].scatter(x, y, c=path_scores, vmin=0.0)
        splt4 = axs[1, 0].scatter(x, y, c=grasp_scores, vmin=0.0)
        splt5 = axs[1, 1].scatter(x, y, c=weighted_cost)
        splt6 = axs[1, 2].scatter(x, y, c=evaluated)

        axs[0, 0].set_title("Uncertainties")
        axs[0, 1].set_title("Costs")
        axs[0, 2].set_title("Path Scores")
        axs[1, 0].set_title("Grasp Scores")
        axs[1, 1].set_title("Weighted Cost")
        axs[1, 2].set_title("Evaluated")

        fig.colorbar(splt1, ax=axs[0,0])
        fig.colorbar(splt2, ax=axs[0,1])
        fig.colorbar(splt3, ax=axs[0,2])
        fig.colorbar(splt4, ax=axs[1,0])
        fig.colorbar(splt5, ax=axs[1,1])
        fig.colorbar(splt6, ax=axs[1,2])

        if folder:
            plt.savefig(f"{folder}/plts/{time.time()}_plt.png")
        
    def choose_best(self) -> ProblemPoints:
        # self.node.get_logger().info("Choose Best")
        cost = [x.cost(self.max_path_score) for x in self.points]
        max_cost = max(cost)
        cost = [(1 if x == max_cost else 0) for x in cost]
        return random.choices(self.points, weights=cost)[0]
        
    def update_map(self):
        # self.node.get_logger().info("Update Map")
        for idx, p in enumerate(self.points):
            orig_certainty = p.certainty
            orig_prediction = p.path_score
            p.update_certainty_by_kde(self.points, logger=self.node.get_logger())
            p.update_prediction_by_kde(self.points, self.max_path_score, logger=self.node.get_logger())
            # self.node.get_logger().info(f"idx {idx}: Updated uncertainty from {orig_certainty} to {p.certainty}")
            # self.node.get_logger().info(f"idx {idx}: Updated path score from {orig_prediction} to {p.path_score}")
        
    def update_max_path_score(self, newest_score):
        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = 2*newest_score
        elif 2*newest_score > self.max_path_score:
            self.max_path_score = 2*newest_score

    @staticmethod
    def validity_failure_model(t: float):
        # calculates the probability that the failure was real
        return float(scipy.stats.weibull_min.cdf(ProblemSpace.weibull_base**t - 1, ProblemSpace.weibull_k, scale=ProblemSpace.weibull_gamma))
            
    def _handle_validity_send_goal(self, context):        
        self.node.get_logger().info("Sending Goal")
        idx, point = self.select_next()

        if point is None:
            context.all_points_certain = True
            if context.pending_results == 0:
                self._handle_validity_finish(context)
            return

        grasp = point.grasp
        
        if point.evaluated:
            self.node.get_logger().info(f"Candidate Grasp (Re-eval) {idx + 1}/{len(self.points)}")
        else:
            self.node.get_logger().info(f"Candidate Grasp {idx + 1}/{len(self.points)}")
                
        self.node.grasp_validity_client_.wait_for_server()
        context.pending_results += 1

        grasp.start_timer()

        send_goal_future = self.node.grasp_validity_client_.send_goal_async(grasp.validity_request_goal(context.request))
        send_goal_future.add_done_callback(lambda f: self._handle_validity_response(f, context, idx))
            
    def _handle_validity_response(self, future, context: ValidityContext, idx):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.node.get_logger().warn("Goal Rejected")
            context.pending_results -= 1
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f : self._handle_validity_result(f, context, idx))

    def _handle_validity_result(self, future, context: ValidityContext, idx):
        result = future.result().result
        hts_grasp : HTSGrasp = self.grasps._grasps[idx]
        hts_grasp.process_result(result)
        hts_grasp.end_timer()
        context.pending_results -= 1
        
        point = self.points[idx]
        point.handle_evaluation_result(result)
        self.node.get_logger().info(f"------- Evaluated idx {idx} ----------")
        cost = point.cost(math.inf)
        self.node.get_logger().info(f"Certainty is now {point.certainty}, Path Score is now {point.path_score}, Cost {cost}")

        self.update_max_path_score(point.path_score)

        self.update_map()

        if self.node.SAVE_DATA:
            hts_grasp.save_grasp_message(context.folder)
            hts_grasp.save_grasp_validity(context.folder, idx == 0)

            # show the updated map
            self.plot_grasps(context.folder)
        
        t1 = time.time()
        self.node.get_logger().info(f"start time is {self.t0} now is {t1} max time is {self.total_max_time} pending {context.pending_results}")
        if context.pending_results == 0 and ((t1 - self.t0) > self.total_max_time or context.all_points_certain):
            self.node.get_logger().info("Finishing Context")
            self._handle_validity_finish(context)
        elif not ((t1 - self.t0) > self.total_max_time) and not (context.all_points_certain):
            self.node.get_logger().info("Timed Out...")
            self._handle_validity_send_goal(context)
        else:
            self.node.get_logger().info("Waiting for others to finish...")

    def _handle_validity_finish(self, context: ValidityContext):
        request = context.goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()

        hts_grasp_group = context.hts_grasp_group
        folder = context.folder
        cloud = context.cloud

        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        context.goal_handle.publish_feedback(feedback)

        if self.node.SAVE_DATA:
            hts_grasp_group.save_metrics(folder)

        hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
        hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
        hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
        hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
        hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        if hts_grasp_group.num_valid():
            self.node.get_logger().info("Found the best grasp")

            # best_grasp = hts_grasp_group.best_grasp()
            best_grasp = self.choose_best().grasp
            AnyGraspNode.display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

            response.grasp_pose = best_grasp.get_pose()
            self.node.get_logger().info("--> " + str(response.grasp_pose))
            response.success = True
            context.goal_handle.succeed()
            context.response = response
        else:
            self.node.get_logger().info("No valid grasps found")
            response.success = False
            context.goal_handle.abort()
            context.response = response
            
class ProblemPoints():
    INITIAL_PLANNING_TIME = 0.03
    
    def __init__(self, grasp: HTSGrasp):              
        self.known_certainty: float = 0 # the known certainty of the path score
        self.certainty: float = 0 # how certain we are of the path score
        self.grasp: HTSGrasp = grasp # grasp oject
        
        roll, pitch, yaw = Rotation.from_quat([
            grasp.get_pose().orientation.x,
            grasp.get_pose().orientation.y,
            grasp.get_pose().orientation.z,
            grasp.get_pose().orientation.w            
        ]).as_euler('xyz', degrees=False)
        self.theta: float = yaw

        self.z: float = grasp.get_pose().position.z
        
        self.is_reflected: bool # whether we are dealing in the reflected space or not
        self.grasp_score: float = grasp.get_grasp_object().score # the score output from anygrasp
        
        self.path_score: float = math.inf # the computed or predicted path length
        self.max_planning_time: float = ProblemPoints.INITIAL_PLANNING_TIME # how long we give for planning
        self.max_planning_time_move: float = 0.03 # how long we give for planning
        
        self.valid: bool = False
        self.evaluated: bool = False
    
    def get_certainty(self):
        return self.certainty
    
    def update_certainty_upon_eval(self):
        if self.valid:
            self.certainty = 1.0
            self.known_certainty = 1.0
        else:
            self.known_certainty = ProblemSpace.validity_failure_model(self.max_planning_time)
            self.certainty = self.known_certainty
            self.max_planning_time *= 2 # double the planning time next time
    
    def cost(self, max_path_score):
        # if we are invalid, we say that the cost is twice is longest path cost
        return -ProblemSpace.lambda1*min(self.path_score, max_path_score) + ProblemSpace.lambda2*self.grasp_score
 
    @functools.cache
    @staticmethod
    def _certainty_scaler_at_offset(z_offset: float, theta_offset: float):
        # we multiply our certainty with a scaled gaussian
        z_scaler = 2*scipy.stats.norm.sf(z_offset, scale=ProblemSpace.sigma_z)
        theta_scaler = 2*scipy.stats.norm.sf(theta_offset, scale=ProblemSpace.sigma_theta)        
        return float(z_scaler * theta_scaler)
    
    @functools.cache
    @staticmethod
    def _distance_scaler(z_offset: float, theta_offset: float):
        z_scale = max(1 - abs(z_offset)/ProblemSpace.prediction_z_range, 0)
        th_scale = max(1 - abs(theta_offset)/ProblemSpace.prediction_th_range, 0)
        # return math.sqrt(z_scale**2 + th_scale**2)
        if z_scale*th_scale == 0.0:
            return 0.0
        
        return float(z_scale*th_scale/2 + 0.5)
    
    def update_certainty_by_kde(self, points: list[ProblemPoints], logger=None):
        if self.evaluated and self.valid:
            # if we have already evaluated this positively, then do not update
            return
        
        # if we have already evaluated this negatively, then we update normally
        if self.evaluated and not self.valid:
            pass

        # if we know nothing about this point, then we use KDE
        # of if we have tried and failed
        positive_contributions = 0.0
        positive_weights = 0.0
        negative_contributions = 0.0
        negative_weights = 0.0
        for i, p in enumerate(points):
            # if p == self:
                # we seed it with the current estimate of certainty
                # all_contributions += 1.0 * self.certainty
                # all_weights += 1.0
                # continue
            if not p.evaluated and p != self:
                continue
            dtheta = abs(p.theta - self.theta) % (2*math.pi)
            dtheta = min(dtheta, 2*math.pi - dtheta)
            
            # this is how much an evaluated point is related to our point
            k = ProblemPoints._certainty_scaler_at_offset(abs(p.z - self.z), dtheta)

            # # don't add a contribution if it's too far away
            # if (k < ProblemSpace.certainty_scaler_range):
            #     continue

            # we store the positive and the negative contributions separately
            if p.valid:
                positive_weights += k
                positive_contributions += k*p.known_certainty
            else:
                negative_weights += k
                if p.evaluated:
                    negative_contributions += k*p.known_certainty
                else:
                    negative_contributions += k*p.certainty

        total_weights = (positive_weights + negative_weights)
        total_contributions = abs(positive_contributions - negative_contributions)
            
        self.certainty = total_contributions/total_weights if total_weights != 0.0 else 0.0
    
    def update_prediction_by_kde(self, points: list[ProblemPoints], path_max, logger=None):
        if self.evaluated and self.valid:
            # if we have already evaluated this positively, then do not update
            return
        
        # if we have already evaluated this negatively, then we update normally
        if self.evaluated and not self.valid:
            pass

        # we start by seeding it with the current prediction
        all_contributions = 0.0
        all_weights = 0.0
        for i, p in enumerate(points):
            if not p.evaluated and p != self:
                continue
            k = ProblemPoints._distance_scaler(p.z - self.z, p.theta - self.theta)
            
            # scale all weights by by the certainty of the point
            if p.evaluated:
                all_weights += k*p.known_certainty
                all_contributions += k*min(p.path_score, path_max)*p.known_certainty

            # print(f"Adding contribution from {i}: distance scaler {k}")
        self.path_score = all_contributions/all_weights if all_weights != 0.0 else path_max*2
        if self.path_score == 0:
            self.path_score = 2*path_max

        # scale in the existing knowledge
        self.path_score = self.path_score*(1 - self.known_certainty) + self.known_certainty*2*path_max
    
    def handle_evaluation_result(self, result: ComputeGraspValidity.Result):
        self.evaluated = True
        self.valid = result.is_valid
        self.update_certainty_upon_eval()
        self.path_score = result.score if result.is_valid else math.inf

class ValidityContext():
    def __init__(self, goal_handle, grasp_group: HTSGraspGroup, folder, cloud, request):
        self.goal_handle = goal_handle
        self.hts_grasp_group = grasp_group
        self.pending_results = 0
        self.folder = folder
        self.cloud = cloud
        self.request = request
        self.response = None
        self.all_points_certain = False

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
        self.SAVE_DATA = self.get_parameter('save_data').value
        self.SYMMETRY_ENABLE = self.get_parameter('symmetry_enable').value
        self.SYMMETRY_LAYER_HEIGHT = self.get_parameter('symmetry_layer_height').value
        self.SYMMETRY_ROTATION_STEP = self.get_parameter('symmetry_rotation_step').value
        self.SYMMETRY_SIMILARITY_THRESHOLD = self.get_parameter('symmetry_similarity_threshold').value

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
        try:
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
        except Exception as e:
            pass

    def display_pointcloud(points, colors=None, save=False, filename=None, origin_position=[0,0,0], description=""):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors) 
        if save:
            o3d.io.write_point_cloud(f"{filename}.pcd", pcd, write_ascii=True)
            np.savez(f"{filename}.npz", points=points, colors=colors)
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

        if self.SAVE_DATA:
            np.savez(f"src/pointclouds/displayed_cloud_{time.time()}.npz", points=points, colors=colors)

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

    def generate_pose_(self, x, y, z, radius, save_folder):
        if self.NO_RGB:
            if not self.POINTCLOUD_FROM_FILE:
                points, colors = self.fast_norgb_pc2_to_numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colors = np.zeros_like(points, dtype=np.float32)
            if self.VISUALISE:
                AnyGraspNode.display_pointcloud(points, save=self.SAVE_DATA, filename=f"{save_folder}/full_cloud", description="Full Point Cloud")
        else:
            if not self.POINTCLOUD_FROM_FILE:
                points, colors = self.fast_pc2_to_numpy(self.depth_pointcloud_)
            else:
                points = self.file_points
                colors = self.file_colors
            if self.VISUALISE:
                AnyGraspNode.display_pointcloud(points, colors, save=self.SAVE_DATA, filename=f"{save_folder}/full_cloud", description="Full Point Cloud")

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
                AnyGraspNode.display_pointcloud(cropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                AnyGraspNode.display_pointcloud(uncropped_points, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")
            else:
                AnyGraspNode.display_pointcloud(cropped_points, cropped_colors, save=self.SAVE_DATA, filename=f"{save_folder}/cropped_cloud", description="Cropped Point Cloud")
                AnyGraspNode.display_pointcloud(uncropped_points, uncropped_colors, save=self.SAVE_DATA, filename=f"{save_folder}/uncropped_cloud", description="Uncropped Point Cloud")

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
        if self.VISUALISE:
            AnyGraspNode.display_grasps(unfiltered_gg, rainbow_cloud, origin_position=[x,y,z], description="All Grasps")

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
            if self.VISUALISE:
                AnyGraspNode.display_grasps(unfiltered_gg, cloud, origin_position=[x,y,z], description="Post-Symmetry Grasps")

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

        # visualization
        if self.VISUALISE:
            AnyGraspNode.display_grasps(gg, cloud, origin_position=[x,y,z], description="Filtered Grasps")
            AnyGraspNode.display_grasps(gg, cloud, only_first=True, origin_position=[x,y,z], description="Highest Grasp Score")

        return gg, cloud

    def log_anygrasp_data(self, f):
        f.write(f"z-coords point cloud bounding [{self.Z_COORDS_MIN}, {self.Z_COORDS_MAX}]\r\n")
        f.write(f"grasping bounds [{self.X_GRASP_MIN},{self.X_GRASP_MAX},{self.Y_GRASP_MIN},{self.Y_GRASP_MAX},{self.Z_GRASP_MIN},{self.Z_GRASP_MAX}]\r\n")
        f.write(f"apply: object mask [{self.APPLY_OBJECT_MASK}] collisions [{self.APPLY_COLLISIONS}] dense_grasp [{self.DENSE_GRASP}]\r\n")
        f.write(f"NMS thresholds: translation [{self.NMS_TRANSLATION_THRESH}] rotation [{self.NMS_ANGLE_THRESH_DEG}]\r\n")
        f.write(f"max pitch/roll filtering: [{self.MAX_GRASP_PITCH_ROLL_DEG}] degrees\r\n")
        f.write(f"grasp offsets: approach axis [{self.GRASP_AXIS_OFFSET}] vertical [{self.GRASP_Z_OFFSET}]\r\n")

    # def _handle_validity_send_goal(self, context, idx):
    #     self.get_logger().info(f"Candidate Grasp {idx + 1}/{len(context.hts_grasp_group)}")
    #     grasp = context.hts_grasp_group._grasps[idx]
    #     grasp.start_timer()
    #     self.grasp_validity_client_.wait_for_server()
    #     send_goal_future = self.grasp_validity_client_.send_goal_async(grasp.validity_request_goal(context.request))
    #     send_goal_future.add_done_callback(lambda f: self._handle_validity_response(f, context, idx))

    # def _handle_validity_response(self, future, context: ValidityContext, idx):
    #     goal_handle = future.result()
        
    #     if not goal_handle.accepted:
    #         self.get_logger().warn("Goal Rejected")
    #         return
        
    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(lambda f : self._handle_validity_result(f, context, idx))

    # def _handle_validity_result(self, future, context: ValidityContext, idx):
    #     result = future.result().result
    #     hts_grasp : HTSGrasp = context.hts_grasp_group._grasps[idx]
    #     hts_grasp.process_result(result)
    #     hts_grasp.end_timer()
    #     context.pending_results -= 1

    #     if self.SAVE_DATA:
    #         hts_grasp.save_grasp_message(context.folder)
    #         hts_grasp.save_grasp_validity(context.folder, idx == 0)
        
    #     if context.pending_results == 0:
    #         self._handle_validity_finish(context)
    #     else:
    #         self._handle_validity_send_goal(context, idx + 1)

    # def _handle_validity_finish(self, context: ValidityContext):
    #     request = context.goal_handle.request
    #     feedback = RequestGrasp.Feedback()
    #     response = RequestGrasp.Result()

    #     hts_grasp_group = context.hts_grasp_group
    #     folder = context.folder
    #     cloud = context.cloud

    #     feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
    #     context.goal_handle.publish_feedback(feedback)

    #     if self.SAVE_DATA:
    #         hts_grasp_group.save_metrics(folder)

    #     hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
    #     hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
    #     hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
    #     hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

    #     if hts_grasp_group.num_valid():
    #         self.get_logger().info("Found the best grasp")

    #         best_grasp = hts_grasp_group.best_grasp()
    #         AnyGraspNode.display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

    #         response.grasp_pose = best_grasp.get_pose()
    #         self.get_logger().info("--> " + str(response.grasp_pose))
    #         response.success = True
    #         context.goal_handle.succeed()
    #         context.response = response
    #     else:
    #         self.get_logger().info("No valid grasps found")
    #         response.success = False
    #         context.goal_handle.abort()
    #         context.response = response

    def grasp_callback_(self, goal_handle):
        request = goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()
            
        folder = f"/ros2_ws/src/out/{time.time()}"
        if self.SAVE_DATA:
            os.makedirs(folder)            
            os.makedirs(f"{folder}/plts")
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
            hts_grasp = HTSGrasp()
            hts_grasp.set_grasp_object(grasp)
            hts_grasp.set_pose(self.map_grasp(grasp, flip_z=False))

            if self.SAVE_DATA:
                hts_grasp.save_grasp_info(folder)

            hts_grasp_group.append(hts_grasp)

        for ind, grasp in enumerate(gg):            
            hts_grasp = HTSGrasp()
            hts_grasp.set_grasp_object(grasp)
            hts_grasp.set_pose(self.map_grasp(grasp, flip_z=True))

            if self.SAVE_DATA:
                hts_grasp.save_grasp_info(folder)

            hts_grasp_group_mirrored.append(hts_grasp)

        context = ValidityContext(goal_handle, hts_grasp_group, folder, cloud, request)
        problem = ProblemSpace(self, hts_grasp_group)
        problem.start_timer()
        problem._handle_validity_send_goal(context)
        # self._handle_validity_send_goal(context, 0)

        while context.response is None:
            time.sleep(0.01)
        time.sleep(10.0)

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
        flip_rotation = Rotation.from_euler('z', 180, degrees=True)
        _, _, yaw = final_rotation.as_euler('xyz', degrees=True)
        if flip_z:
            self.get_logger().info("Identified Flipped Grasp. Unflipping...")
            final_rotation = final_rotation * flip_rotation

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