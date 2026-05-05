
from __future__ import annotations
import time
import math
import numpy as np
import time
import random
import matplotlib.pyplot as plt

from hts_msgs.action import ComputeGraspValidity, RequestGrasp

from .hts_grasps import HTSGrasp, HTSGraspGroup
from .utils import display_grasps, ValidityContext

# construct a problem space
class GraspSelectorBasic():
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
        
        self.points: list[ProblemPointsBasic] = [ProblemPointsBasic(g) for g in hts_gg._grasps] # points
        self.t0 = 0.0

        self.max_path_score = 0.0

        zs = np.array([p.z for p in self.points])
        thetas = np.array([p.theta for p in self.points])
        np.save("/ros2_ws/src/z", zs)
        np.save("/ros2_ws/src/th", thetas)

        # self.plot_grasps()
    
    def start_timer(self):
        self.t0 = time.time()
    
    def select_next(self):
        if self.num_iterations >= len(self.points):
            return 0, None
        return self.num_iterations, self.points[self.num_iterations]
        
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
            filename = f"{folder}/plts/{time.time()}_plt.png" if not is_flipped else f"{folder}/plts/flipped_{time.time()}_plt.png"
            plt.savefig(filename)
        
    def choose_best(self) -> ProblemPointsBasic:
        # self.logger.info("Choose Best")
        cost = [x.cost(self.max_path_score) for x in self.points]
        max_cost = max(cost)
        cost = [(1 if x == max_cost else 0) for x in cost]
        return random.choices(self.points, weights=cost)[0]
        
    def update_map(self):
        pass
        
    def update_max_path_score(self, newest_score):
        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = 2*newest_score
        elif 2*newest_score > self.max_path_score:
            self.max_path_score = 2*newest_score

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
            hts_grasp.save_grasp_validity(context.folder, self.num_iterations == 1, context.is_flipped)

            # show the updated map
            self.plot_grasps(context.folder, context.is_flipped)
        
        t1 = time.time()
        self.logger.info(f"start time is {self.t0} now is {t1} max time is {self.total_max_time} pending {context.pending_results}")
        if context.pending_results == 0 and (context.all_points_certain):
            self.logger.info("Finishing Context")
            self._handle_validity_finish(context)
        elif not (context.all_points_certain):
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

        if context.visualise:
            hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
            hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
            hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        if hts_grasp_group.num_valid():
            self.logger.info("Found the best grasp")

            # best_grasp = hts_grasp_group.best_grasp()
            best_grasp = self.choose_best().grasp
            if context.visualise:
                display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

            response.grasp_pose = best_grasp.get_pose()
            self.logger.info("--> " + str(response.grasp_pose))
            response.success = True
            context.response = response
        else:
            self.logger.info("No valid grasps found")
            response.success = False
            context.response = response