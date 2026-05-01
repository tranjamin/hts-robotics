
from __future__ import annotations
import time
import math
import numpy as np
import time
import scipy.stats
import functools
import random
import matplotlib.pyplot as plt
from sklearn.gaussian_process.kernels import Matern
from hts_msgs.action import ComputeGraspValidity, RequestGrasp
import matplotlib
matplotlib.use("svg")

from .hts_grasps import HTSGrasp, HTSGraspGroup
from .utils import display_grasps

# construct a problem space
class GraspSelectorGP():
    lambda1: float = 1.0
    lambda2: float = 30.0
    
    weibull_k: float = 0.7
    weibull_gamma: float = 0.5
    weibull_base: float = 1.3
    
    total_max_time = 20.0

    num_iterations = 0

    kappa = 3
    kernel = Matern()
    
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
        np.save("/ros2_ws/src/z", zs)
        np.save("/ros2_ws/src/th", thetas)
        
        self.N = len(self.points)
        self.k = 1.0

        self.all_z = np.array([p.z for p in self.points]).reshape((len(self.points), 1))
        self.all_th = np.array([p.theta for p in self.points]).reshape((len(self.points), 1))
        self.all_coords = np.hstack((self.all_z, self.all_th))

        self.mean_cost = np.zeros((self.N, 1))
        self.cov = np.eye(self.N)

        # self.plot_grasps()
    
    def start_timer(self):
        self.t0 = time.time()
    
    def select_next(self):
        # construct training dataset from already sampled points
        weights = (self.mean_cost.ravel() + GraspSelectorGP.kappa*np.sqrt(np.diag(self.cov)))*np.array([1 - p.certainty for p in self.points])
        weights = weights - np.min(weights)

        # weights = (self.mean_cost.ravel() + 3*np.sqrt(np.diag(self.cov)))
        # weights = weights - np.min(weights)

        if np.sum(weights) == 0.0:
            idx = random.choices(range(len(self.points)))[0]
            return idx, self.points[idx]

        # sample
        idx = random.choices(range(len(self.points)), weights=list(weights))[0]
        return idx, self.points[idx]
        
    def plot_grasps(self, folder=None, is_flipped=False):
        zs = np.array([p.z for p in self.points])
        thetas = np.array([p.theta for p in self.points])

        x = zs*np.cos(thetas)
        y = zs*np.sin(thetas)

        x_mesh, y_mesh = np.meshgrid(np.linspace(-0.1, 0.1, 60), np.linspace(-0.1, 0.1, 60))
        x_mesh = np.array(x_mesh).reshape((-1, 1))
        y_mesh = np.array(y_mesh).reshape((-1, 1))
        z_mesh = np.sqrt(x_mesh**2 + y_mesh**2)
        th_mesh = np.arctan2(y_mesh, x_mesh)

        mesh_mean, mesh_cov = self.get_preds(np.hstack((z_mesh, th_mesh)))
        mesh_uncertanties = np.sqrt(np.diag(mesh_cov))
        # mesh_mean = self.max_path_score - mesh_mean
        x_mesh = z_mesh*np.cos(th_mesh)
        y_mesh = z_mesh*np.sin(th_mesh)

        costs_estimated = self.mean_cost        
        uncertanties = np.sqrt(np.diag(self.cov))

        evaluated = [x.evaluated for x in self.points]

        true_costs = [x.cost(self.max_path_score) for x in self.points]

        grasp_scores = [x.grasp_score for x in self.points]
        calculated_path_scores = [0.0 if not x.evaluated else x.path_score for x in self.points]

        weights = (self.mean_cost.ravel() - self.k*np.sqrt(np.diag(self.cov)))

        fig, axs = plt.subplots(3, 3, figsize=(24, 12))
        splt1 = axs[0, 0].scatter(x, y, c=uncertanties)
        splt2 = axs[0, 1].scatter(x, y, c=costs_estimated)
        splt3 = axs[0, 2].scatter(x, y, c=true_costs)

        splt4 = axs[1, 0].scatter(x_mesh, y_mesh, c=mesh_uncertanties)
        splt5 = axs[1, 1].scatter(x_mesh, y_mesh, c=mesh_mean)
        splt6 = axs[1, 2].scatter(x, y, c=calculated_path_scores)

        splt7 = axs[2, 0].scatter(x, y, c=grasp_scores)
        splt8 = axs[2, 1].scatter(x, y, c=weights)
        splt9 = axs[2, 2].scatter(x, y, c=evaluated)

        axs[0, 0].set_title("Variance")
        axs[0, 1].set_title("Mean Cost Score")
        axs[0, 2].set_title("Calculateed Costs Scores")

        axs[1, 0].set_title("Variance Map")
        axs[1, 1].set_title("Mean Cost Map")
        axs[1, 2].set_title("Calculated Path Scores")

        axs[2, 0].set_title("Grasp Scores")
        axs[2, 1].set_title("Sampling Weights")
        axs[2, 2].set_title("Evaluated")

        fig.colorbar(splt1, ax=axs[0,0])
        fig.colorbar(splt2, ax=axs[0,1])
        fig.colorbar(splt3, ax=axs[0,2])
        fig.colorbar(splt4, ax=axs[1,0])
        fig.colorbar(splt5, ax=axs[1,1])
        fig.colorbar(splt6, ax=axs[1,2])
        fig.colorbar(splt7, ax=axs[2,0])
        fig.colorbar(splt8, ax=axs[2,1])
        fig.colorbar(splt9, ax=axs[2,2])


        # plt.savefig(f"{time.time()}_plt.png")
        # plt.show()

        if folder:
            filename = f"{folder}/plts/{time.time()}_plt.svg" if not is_flipped else f"{folder}/plts/flipped_{time.time()}_plt.svg"
            plt.savefig(filename)
        
    def choose_best(self) -> tuple[ProblemPoints | None, float]:
        # self.logger.info("Choose Best")
        best_cost = 0.0
        best_point = None

        for p in self.points:
            if not p.evaluated:
                continue
            cost = p.cost(self.max_path_score)
            if cost > best_cost:
                best_cost = cost
                best_point = p

        return best_point, best_cost
        
    def update_map(self):
        # construct training dataset from already sampled points
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))
        evaluated_noise = np.array([1 - p.certainty for p in evaluated_points])

        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(self.all_coords, evaluated_coords)
        Kxx = self.kernel(self.all_coords)

        # Sigma = np.eye(len(evaluated_points))
        Sigma = np.diag([(1 - p.known_certainty)*np.max(self.cov) for p in evaluated_points])
        y = np.array([p.cost(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        
        self.mean_cost = KxX @ np.linalg.inv(KXX + Sigma) @ y
        self.cov = Kxx - KxX @ np.linalg.inv(KXX + Sigma) @ KxX.T

        np.clip(self.cov, 0.0, None, self.cov)

    def get_preds(self, coords):
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))
        evaluated_noise = np.array([1 - p.certainty for p in evaluated_points])

        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(coords, evaluated_coords)
        Kxx = self.kernel(coords)

        y = np.array([p.cost(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        u = KxX @ np.linalg.inv(KXX) @ y
        c = Kxx - KxX @ np.linalg.inv(KXX) @ KxX.T
        return u,c
        
    def update_max_path_score(self, newest_score):
        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = newest_score
        elif newest_score > self.max_path_score:
            self.max_path_score = newest_score

    @staticmethod
    def validity_failure_model(t: float):
        # calculates the probability that the failure was real
        return float(scipy.stats.weibull_min.cdf(GraspSelectorGP.weibull_base**t - 1, GraspSelectorGP.weibull_k, scale=GraspSelectorGP.weibull_gamma))
            
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

    def _handle_validity_result(self, future, context: ValidityContext, idx, ):
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
        if context.pending_results == 0 and ((t1 - self.t0) > self.total_max_time or context.all_points_certain):
            self.logger.info("Finishing Context")
            self._handle_validity_finish(context)
        elif not ((t1 - self.t0) > self.total_max_time) and not (context.all_points_certain):
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
            best_point, best_cost = self.choose_best()

            if best_point is None:
                self.logger.info("No valid grasps found")
                response.success = False
                context.goal_handle.abort()
                context.response = response
            best_grasp = best_point.grasp
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
            self.known_certainty = GraspSelectorGP.validity_failure_model(self.max_planning_time)
            self.certainty = self.known_certainty
            self.max_planning_time *= 2 # double the planning time next time
    
    def cost(self, max_path_score):
        # if we are invalid, we say that the cost is twice is longest path cost
        return max(max_path_score - self.path_score, -max_path_score)*self.grasp_score
 
    def handle_evaluation_result(self, result: ComputeGraspValidity.Result):
        self.evaluated = True
        self.valid = result.is_valid
        self.update_certainty_upon_eval()
        self.path_score = result.score if result.is_valid else math.inf

class DualGraspSelectorGP():
    def __init__(self, group1: HTSGraspGroup, group2: HTSGraspGroup, context1, context2, final_context, logger, client):
        self.selector1 = GraspSelectorGP(group1, logger, client)
        self.selector2 = GraspSelectorGP(group2, logger, client)

        self.selector1.start_timer()
        self.selector2.start_timer()

        self.context1 = context1
        self.context2 = context2
        self.final_context = final_context
        self.logger = logger
        self.client = client

        self.first_terminated = False
        self.first_bestgrasp = None
        self.first_bestcost = 0
        self.second_bestgrasp = None
        self.second_bestcost = 0

        self.current_selector = self.selector2
        self.current_context = self.context2

    def _handle_validity_send_goal(self):
        self.current_selector = self.selector2 if self.current_selector == self.selector1 else self.selector1
        self.current_context = self.context2 if self.current_context == self.context1 else self.context1
        self.logger.info("FLIPPING>>>>")

        self.logger.info("Sending Goal")
        idx, point = self.current_selector.select_next()

        if point is None:
            self.logger.info("POINT IS NONE")
            self.current_context.all_points_certain = True
            if self.current_context.pending_results == 0:
                self._handle_validity_finish()
            return

        grasp = point.grasp
        
        if point.evaluated:
            self.logger.info(f"Candidate Grasp (Re-eval) {idx + 1}/{len(self.current_selector.points)}")
        else:
            self.logger.info(f"Candidate Grasp {idx + 1}/{len(self.current_selector.points)}")
                
        self.client.wait_for_server()
        self.current_context.pending_results += 1

        grasp.start_timer()

        send_goal_future = self.client.send_goal_async(grasp.validity_request_goal(self.current_context.request, point.max_planning_time))
        send_goal_future.add_done_callback(lambda f: self._handle_validity_response(f, idx))

    def _handle_validity_response(self, future, idx):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.warn("Goal Rejected")
            self.current_context.pending_results -= 1
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f : self._handle_validity_result(f, idx))

    def _handle_validity_result(self, future, idx):
        result = future.result().result
        hts_grasp : HTSGrasp = self.current_selector.grasps._grasps[idx]
        hts_grasp.process_result(result)
        hts_grasp.end_timer()
        self.current_context.pending_results -= 1
        self.current_selector.num_iterations += 1
        
        point = self.current_selector.points[idx]
        point.handle_evaluation_result(result)
        self.logger.info(f"------- Evaluated idx {idx} ----------")
        cost = point.cost(math.inf)
        self.logger.info(f"Certainty is now {point.certainty}, Path Score is now {point.path_score}, Cost {cost}")

        self.current_selector.update_max_path_score(point.path_score)

        self.current_selector.update_map()

        if self.current_selector.SAVE_DATA:
            hts_grasp.save_grasp_message(self.current_context.folder, self.current_context.is_flipped)
            hts_grasp.save_grasp_validity(self.current_context.folder, self.current_selector.num_iterations == 1, self.current_context.is_flipped)

            # show the updated map
            self.current_selector.plot_grasps(self.current_context.folder, self.current_context.is_flipped)
        
        t1 = time.time()
        self.logger.info(f"start time is {self.current_selector.t0} now is {t1} max time is {self.current_selector.total_max_time} pending {self.current_context.pending_results}")
        if self.current_context.pending_results == 0 and ((t1 - self.current_selector.t0) > self.current_selector.total_max_time or self.current_context.all_points_certain):
            self.logger.info("Finishing Context")
            self._handle_validity_finish()
        elif not ((t1 - self.current_selector.t0) > self.current_selector.total_max_time) and not (self.current_context.all_points_certain):
            self._handle_validity_send_goal()
        else:
            self.logger.info("Waiting for others to finish...")

    def _handle_validity_finish(self):
        self.logger.info(f"Handling Validity Finish for {self.current_selector == self.selector1}")
        request = self.current_context.goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()

        hts_grasp_group = self.current_context.hts_grasp_group
        folder = self.current_context.folder
        cloud = self.current_context.cloud

        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        self.current_context.goal_handle.publish_feedback(feedback)

        if self.current_selector.SAVE_DATA:
            hts_grasp_group.save_metrics(folder, self.current_context.is_flipped)

        hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
        hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
        hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
        hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
        hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        if hts_grasp_group.num_valid():
            self.logger.info("Found the best grasp")

            best_point, best_cost = self.current_selector.choose_best()

            if best_point is None:
                self.logger.info("No valid grasps found")
                response.success = False

            best_grasp = best_point.grasp
            
            if not self.first_terminated:
                self.first_bestcost = best_cost
                self.first_bestgrasp = best_grasp
            else:
                self.second_bestcost = best_cost
                self.second_bestgrasp = best_grasp

            display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

            response.grasp_pose = best_grasp.get_pose()
            self.logger.info("--> " + str(response.grasp_pose))
            response.success = True
        else:
            self.logger.info("No valid grasps found")
            response.success = False

        self.current_context.response = response

        if self.first_terminated:
            final_response = RequestGrasp.Result()
            final_response.success = self.context1.response.success or self.context2.response.success
            if (self.first_bestgrasp is not None and self.second_bestgrasp is not None):
                if self.first_bestcost > self.second_bestcost:
                    final_response.grasp_pose = self.first_bestgrasp.get_pose()
                else:
                    final_response.grasp_pose = self.second_bestgrasp.get_pose()
            elif (self.first_bestgrasp is not None):
                final_response.grasp_pose = self.context1.response.grasp_pose.get_pose()
            elif (self.second_bestgrasp is not None):
                final_response.grasp_pose = self.context2.response.grasp_pose.get_pose()
            
            if final_response.success:
                self.final_context.goal_handle.succeed()
            else:
                self.final_context.goal_handle.abort()
            self.final_context.response = final_response
        else:
            self.first_terminated = True
            self.current_selector = self.selector2 if self.current_selector == self.selector1 else self.selector1
            self.current_context = self.context2 if self.current_context == self.context1 else self.context1
            self._handle_validity_finish()


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