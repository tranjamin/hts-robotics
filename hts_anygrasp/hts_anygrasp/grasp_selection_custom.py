
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
import scipy.stats
import functools
import random
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("svg")

from ament_index_python.packages import get_package_prefix
from scipy.spatial.transform import Rotation

import rclpy
from hts_msgs.action import ComputeGraspValidity, RequestGrasp


from .hts_grasps import HTSGrasp, HTSGraspGroup
from .utils import display_grasps

# construct a problem space
class GraspSelectorCustom():
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
    
    total_max_time = 500.0

    num_iterations = 0
    
    def __init__(self, hts_gg: HTSGraspGroup, logger, client):
        self.grasps: HTSGraspGroup = hts_gg # all grasps
        self.logger = logger
        self.client = client
        self.SAVE_DATA = True
        
        self.points: list[ProblemPoints] = [ProblemPoints(g) for g in hts_gg._grasps] # points
        self.t0 = 0.0

        self.max_path_score = 0.0

        zs = np.array([p.z for p in self.points])
        thetas = np.array([p.theta for p in self.points])
        # np.save("/ros2_ws/src/z", zs)
        # np.save("/ros2_ws/src/th", thetas)

        # self.plot_grasps()
    
    def start_timer(self):
        self.t0 = time.time()
    
    def select_next(self):
        self.logger.info("Select Next")
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
                self.logger.info(f"Selecting probabilistic greedily {idx}")
                return idx, self.points[idx]
            else:
                # print(f"Trying to sample greedily")
                idx = weighted_cost.index(max(weighted_cost))
                self.logger.info(f"Selecting greedily {idx}")
                return idx, self.points[idx]
        elif sum(uncertanties) != 0.0:
            if random.random() < 0.5:
                # print(f"Trying to sample uncertainty")
                idx = random.choices(range(len(self.points)), weights=uncertanties)[0]
                self.logger.info(f"Selecting greedily based on uncertainty {idx}")
                return idx, self.points[idx]
            else:
                # print(f"Trying to sample greedily")
                idx = uncertanties.index(max(uncertanties))
                self.logger.info(f"Selecting based on uncertainty {idx}")
                return idx, self.points[idx]
        else:
            self.logger.info("Everything is 100%% certain now")
            return 0, None
        
    def plot_grasps(self, folder=None, is_flipped=False):
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
            filename = f"{folder}/plts/{time.time()}_plt.svg" if not is_flipped else f"{folder}/plts/flipped_{time.time()}_plt.svg"
            plt.savefig(filename)
        
    def choose_best(self) -> ProblemPoints:
        # self.logger.info("Choose Best")
        cost = [x.cost(self.max_path_score) for x in self.points]
        max_cost = max(cost)
        cost = [(1 if x == max_cost else 0) for x in cost]
        return random.choices(self.points, weights=cost)[0]
        
    def update_map(self):
        # self.logger.info("Update Map")
        for idx, p in enumerate(self.points):
            orig_certainty = p.certainty
            orig_prediction = p.path_score
            p.update_certainty_by_kde(self.points, logger=self.logger)
            p.update_prediction_by_kde(self.points, self.max_path_score, logger=self.logger)
            # self.logger.info(f"idx {idx}: Updated uncertainty from {orig_certainty} to {p.certainty}")
            # self.logger.info(f"idx {idx}: Updated path score from {orig_prediction} to {p.path_score}")
        
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
        return float(scipy.stats.weibull_min.cdf(GraspSelectorCustom.weibull_base**t - 1, GraspSelectorCustom.weibull_k, scale=GraspSelectorCustom.weibull_gamma))
            
    def _handle_validity_send_goal(self, context):        
        self.logger.info("Sending Goal")
        idx, point = self.select_next()

        if point is None:
            context.all_points_certain = True
            if context.pending_results == 0:
                self._handle_validity_finish(context)
            return

        grasp = point.grasp
        
        if point.evaluated:
            self.logger.info(f"Candidate Grasp (Re-eval) {idx + 1}/{len(self.points)}")
        else:
            self.logger.info(f"Candidate Grasp {idx + 1}/{len(self.points)}")
                
        self.client.wait_for_server()
        context.pending_results += 1

        grasp.start_timer()

        send_goal_future = self.client.send_goal_async(grasp.validity_request_goal(context.request, point.max_planning_time))
        send_goal_future.add_done_callback(lambda f: self._handle_validity_response(f, context, idx))
            
    def _handle_validity_response(self, future, context: ValidityContext, idx):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.warn("Goal Rejected")
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
        self.num_iterations += 1
        
        point = self.points[idx]
        point.handle_evaluation_result(result)
        self.logger.info(f"------- Evaluated idx {idx} ----------")
        cost = point.cost(math.inf)
        self.logger.info(f"Certainty is now {point.certainty}, Path Score is now {point.path_score}, Cost {cost}")

        self.update_max_path_score(point.path_score)

        self.update_map()

        if self.SAVE_DATA:
            hts_grasp.save_grasp_message(context.folder, context.is_flipped)
            hts_grasp.save_grasp_validity(context.folder, self.num_iterations == 0, context.is_flipped)

            # show the updated map
            self.plot_grasps(context.folder, context.is_flipped)
        
        t1 = time.time()
        self.logger.info(f"start time is {self.t0} now is {t1} max time is {self.total_max_time} pending {context.pending_results}")
        if context.pending_results == 0 and ((t1 - self.t0) > self.total_max_time or context.all_points_certain):
            self.logger.info("Finishing Context")
            self._handle_validity_finish(context)
        elif not ((t1 - self.t0) > self.total_max_time) and not (context.all_points_certain):
            self.logger.info("Timed Out...")
            self._handle_validity_send_goal(context)
        else:
            self.logger.info("Waiting for others to finish...")

    def _handle_validity_finish(self, context: ValidityContext):
        request = context.goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()

        hts_grasp_group = context.hts_grasp_group
        folder = context.folder
        cloud = context.cloud

        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        context.goal_handle.publish_feedback(feedback)

        if self.SAVE_DATA:
            hts_grasp_group.save_metrics(folder, context.is_flipped)

        hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
        hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
        hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
        hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
        hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        if hts_grasp_group.num_valid():
            self.logger.info("Found the best grasp")

            # best_grasp = hts_grasp_group.best_grasp()
            best_grasp = self.choose_best().grasp
            display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

            response.grasp_pose = best_grasp.get_pose()
            self.logger.info("--> " + str(response.grasp_pose))
            response.success = True
            context.goal_handle.succeed()
            context.response = response
        else:
            self.logger.info("No valid grasps found")
            response.success = False
            context.goal_handle.abort()
            context.response = response
            
class ProblemPoints():
    INITIAL_PLANNING_TIME = 0.03
    
    def __init__(self, grasp: HTSGrasp):              
        self.known_certainty: float = 0 # the known certainty of the path score
        self.certainty: float = 0 # how certain we are of the path score
        self.grasp: HTSGrasp = grasp # grasp oject
        
        self.z = grasp.z
        self.theta = grasp.theta
        
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
            self.known_certainty = GraspSelectorCustom.validity_failure_model(self.max_planning_time)
            self.certainty = self.known_certainty
            self.max_planning_time *= 2 # double the planning time next time
    
    def cost(self, max_path_score):
        # if we are invalid, we say that the cost is twice is longest path cost
        return -GraspSelectorCustom.lambda1*min(self.path_score, max_path_score) + GraspSelectorCustom.lambda2*self.grasp_score
 
    @functools.cache
    @staticmethod
    def _certainty_scaler_at_offset(z_offset: float, theta_offset: float):
        # we multiply our certainty with a scaled gaussian
        z_scaler = 2*scipy.stats.norm.sf(z_offset, scale=GraspSelectorCustom.sigma_z)
        theta_scaler = 2*scipy.stats.norm.sf(theta_offset, scale=GraspSelectorCustom.sigma_theta)        
        return float(z_scaler * theta_scaler)
    
    @functools.cache
    @staticmethod
    def _distance_scaler(z_offset: float, theta_offset: float):
        z_scale = max(1 - abs(z_offset)/GraspSelectorCustom.prediction_z_range, 0)
        th_scale = max(1 - abs(theta_offset)/GraspSelectorCustom.prediction_th_range, 0)
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
            # if (k < GraspSelectorCustom.certainty_scaler_range):
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
    def __init__(self, goal_handle, grasp_group: HTSGraspGroup, folder, cloud, request, is_flipped:bool = False):
        self.goal_handle = goal_handle
        self.hts_grasp_group = grasp_group
        self.pending_results = 0
        self.folder = folder
        self.cloud = cloud
        self.request = request
        self.response = None
        self.all_points_certain = False
        self.is_flipped = is_flipped