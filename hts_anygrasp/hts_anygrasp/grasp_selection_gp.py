
from __future__ import annotations
import time
import math
import numpy as np
import time
import scipy.stats
import functools
import random
import matplotlib.pyplot as plt
from sklearn.gaussian_process.kernels import Matern, RBF, ExpSineSquared
from hts_msgs.action import ComputeGraspValidity, RequestGrasp
import matplotlib
import matplotlib.cm as cm
matplotlib.use("agg")

from .hts_grasps import HTSGrasp, HTSGraspGroup
from .utils import display_grasps

class PlanningTimeTuner():

    def __init__(self):
        self.planning_time_mean: float = np.log(0.03)
        self.planning_time_variance: float = 0.5
        self.planning_data = np.array([])

    def add_planning_data(self, t: float):
        self.planning_data = np.append(self.planning_data, t)
        self.planning_time_mean = 1/self.planning_data.shape[0]*np.sum(np.log(self.planning_data))
        if self.planning_data.shape[0] > 2:
            self.planning_time_variance = 1/self.planning_data.shape[0]*np.sum(np.square(np.log(self.planning_data) - self.planning_time_mean))

    def validity_failure_model(self, t: float):
        return PlanningTimeTuner._compute_norm(t, self.planning_time_mean, self.planning_time_variance)
        # return 1

    @staticmethod
    def _compute_norm(t: float, mean, var):
        return float(scipy.stats.lognorm.cdf(t, np.sqrt(var), scale=np.exp(mean)))

class CompositeKernel(RBF):
    def __init__(self, distance_kernel, angle_kernel):
        self.distance_kernel = distance_kernel
        self.angle_kernel = angle_kernel

    def __call__(self, X, Y=None, eval_gradient=False):
        X_r = X[:, 0].reshape((-1, 1))
        X_th = X[:, 1].reshape((-1, 1))
        
        if Y is None:
            return self.distance_kernel(X_r, Y=None, eval_gradient=eval_gradient) * self.angle_kernel(X_th, Y=None, eval_gradient=eval_gradient)
        else:
            Y_r = Y[:, 0].reshape((-1, 1))
            Y_th = Y[:, 1].reshape((-1, 1))
            return self.distance_kernel(X_r, Y=Y_r, eval_gradient=eval_gradient) * self.angle_kernel(X_th, Y=Y_th, eval_gradient=eval_gradient)


# construct a problem space
class GraspSelectorGP():
    planning_time_mean: float = 0.3
    planning_time_cov: float = 0.3
    
    total_max_time = 300.0

    num_iterations = 0

    kappa = 3
    
    dist_kernel = Matern(length_scale=0.03, nu=2.5)
    angle_kernel = ExpSineSquared(length_scale=0.8, periodicity=2*np.pi)
    kernel = CompositeKernel(dist_kernel, angle_kernel)

    # kernel = Matern(length_scale=[0.03, 0.8], nu=3.5)
    
    def __init__(self, hts_gg: HTSGraspGroup, logger, client, tuner: PlanningTimeTuner):
        self.grasps: HTSGraspGroup = hts_gg # all grasps
        self.logger = logger
        self.client = client
        self.tuner = tuner
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

        # coordinates used for update map
        self.all_z = np.array([p.z for p in self.points]).reshape((len(self.points), 1))
        self.all_th = np.array([p.theta for p in self.points]).reshape((len(self.points), 1))
        self.all_coords = np.hstack((self.all_z, self.all_th))

        # precompute x and y for plotting
        self.all_x = self.all_z*np.cos(self.all_th)
        self.all_y = self.all_z*np.sin(self.all_th)

        # precompute a mesh for map plotting
        self.x_mesh, self.y_mesh = np.meshgrid(
            np.linspace(np.min(self.all_x), np.max(self.all_x), 80), 
            np.linspace(np.min(self.all_y), np.max(self.all_y), 80)
        )
        self.x_mesh = np.array(self.x_mesh).reshape((-1, 1))
        self.y_mesh = np.array(self.y_mesh).reshape((-1, 1))
        self.z_mesh = np.sqrt(self.x_mesh**2 + self.y_mesh**2)
        self.th_mesh = np.arctan2(self.y_mesh, self.x_mesh)

        mesh_filter = (self.z_mesh >= np.min(self.all_z)) & (self.z_mesh <= np.max(self.all_z))
        self.x_mesh = self.x_mesh[mesh_filter].reshape((-1, 1))
        self.y_mesh = self.y_mesh[mesh_filter].reshape((-1, 1))
        self.z_mesh = self.z_mesh[mesh_filter].reshape((-1, 1))
        self.th_mesh = self.th_mesh[mesh_filter].reshape((-1, 1))

        # parameters for the GP
        self.est_path_scores = np.zeros((self.N, 1))
        self.cov = np.eye(self.N)
        self.est_planning_times = np.ones((self.N, 1))*ProblemPoints.INITIAL_PLANNING_TIME
    
    def start_timer(self):
        self.t0 = time.time()
    
    def select_next(self):
        # weits for sampling
        weights = (self.est_path_scores.ravel() + GraspSelectorGP.kappa*np.sqrt(np.diag(self.cov)))
        weights = weights - np.min(weights)
        weights = weights * np.array([1 - p.certainty for p in self.points])

        self.logger.info(f"Weight Sum: {np.sum(weights)}")

        # choose a random points if no weights
        if np.sum(weights) == 0.0:
            idx = random.choices(range(len(self.points)))[0]
        else:
            idx = random.choices(range(len(self.points)), weights=list(weights))[0]

        return idx, self.points[idx]
        
    def plot_grasps(self, folder=None, is_flipped=False):
        # predict mean and cov for mesh
        tmp1 = time.time()
        mesh_mean, mesh_cov, mesh_times = self.get_preds(np.hstack((self.z_mesh, self.th_mesh)))
        self.logger.info(f"!!!!!!!!! Took {time.time() - tmp1} seconds to get predictions")
        tmp1 = time.time()

        mesh_uncertanties = np.sqrt(np.diag(mesh_cov))

        # get the mean and uncertainty for 
        gp_mean = self.est_path_scores
        gp_uncertainties = np.sqrt(np.diag(self.cov))

        # whether a point has been evaluated or not
        evaluated = [x.evaluated for x in self.points]
        validity = [(1 if x.valid else -1) if x.evaluated else 0 for x in self.points]

        # evaluated filter
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z for p in evaluated_points])
        evaluated_th = np.array([p.theta for p in evaluated_points])
        
        evaluated_paths = [p.norm_path_score(self.max_path_score) for p in evaluated_points]
        evaluated_uncertainty = [1 - p.known_certainty for p in evaluated_points]

        # weights 
        weights = (self.est_path_scores.ravel() + GraspSelectorGP.kappa*np.sqrt(np.diag(self.cov)))
        weights = weights - np.min(weights)
        weights = weights * np.array([1 - p.certainty for p in self.points])

        clipped_planning_times = np.log2(np.clip(self.est_planning_times, ProblemPoints.INITIAL_PLANNING_TIME, None))
        clipped_mesh_times = np.log2(np.clip(mesh_times, ProblemPoints.INITIAL_PLANNING_TIME, None))

        self.logger.info(f"!!!!!!!!! Took {time.time() - tmp1} seconds to make arrays")
        tmp1 = time.time()

        fig, axs = plt.subplots(2, 4, figsize=(30, 12), subplot_kw={'projection': 'polar'})
        splt1 = axs[0, 0].scatter(
            self.all_th, self.all_z, vmin=np.min(gp_mean), vmax=np.max(gp_mean),
            c=gp_mean, cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
            )
        splt2 = axs[0, 1].scatter(
            self.all_th, self.all_z, vmin=0.0, vmax=1.0,
            c=gp_uncertainties, cmap=cm.RdYlGn_r,marker="o", linewidths=0.3, edgecolors="black"
            )
        splt3 = axs[0, 2].scatter(
            self.all_th, self.all_z, vmin=-1.0, vmax=1.0,
            c=validity, cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
            )

        splt4 = axs[1, 0].scatter(
            self.th_mesh, self.z_mesh, vmin=np.min(mesh_mean), vmax=np.max(mesh_mean),
            c=mesh_mean, cmap=cm.RdYlGn,marker=","
            )
        splt5 = axs[1, 1].scatter(
            self.th_mesh, self.z_mesh, vmin=0.0, vmax=1.0,
            c=mesh_uncertanties, cmap=cm.RdYlGn_r,marker=","
            )
        splt6 = axs[1, 2].scatter(
            self.all_th, self.all_z, vmin=0.0, vmax=1.0,
            c=evaluated, cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
            )
        
        splt7 = axs[0, 3].scatter(
            self.all_th, self.all_z, vmin=np.min(weights), vmax=np.max(weights),
            c=weights, cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
            )
        
        splt8 = axs[1, 3].scatter(
            self.th_mesh, self.z_mesh,
            c=clipped_mesh_times - np.log2(ProblemPoints.INITIAL_PLANNING_TIME), cmap=cm.RdYlGn_r,marker=","
            )

        
        axs[1, 0].scatter(
            evaluated_th, evaluated_z, vmin=np.min(mesh_mean), vmax=np.max(mesh_mean),
            c=evaluated_paths, cmap=cm.RdYlGn, marker="o", linewidths=0.3, edgecolors="black"
            )
        axs[1, 1].scatter(
            evaluated_th, evaluated_z, vmin=0.0, vmax=1.0,
            c=evaluated_uncertainty, cmap=cm.RdYlGn_r, marker="o", linewidths=0.3, edgecolors="black"
            )
        axs[1, 3].scatter(
            self.all_th, self.all_z,
            c=clipped_planning_times - np.log2(ProblemPoints.INITIAL_PLANNING_TIME), cmap=cm.RdYlGn_r, marker="o", linewidths=0.3, edgecolors="black"
            )

        axs[0, 0].set_title("Predicted Path Score (Normalised)")
        axs[0, 1].set_title("Path Score Uncertainty")
        axs[0, 2].set_title("Grasp Validity")

        axs[1, 0].set_title("GP Map of Pred. Path Score (Norm.)")
        axs[1, 1].set_title("GP Covariance Map")
        axs[1, 2].set_title("Evaluated Grasps")

        axs[0, 3].set_title("Sampling Weights")
        axs[1, 3].set_title("Planning Time Multiplier")

        axs[0, 0].set_axisbelow(True)
        axs[0, 1].set_axisbelow(True)
        axs[0, 2].set_axisbelow(True)
        axs[0, 3].set_axisbelow(True)
        # axs[1, 0].set_axisbelow(True)
        # axs[1, 1].set_axisbelow(True)
        axs[1, 2].set_axisbelow(True)
        # axs[1, 3].set_axisbelow(True)

        fig.colorbar(splt1, ax=axs[0,0])
        fig.colorbar(splt2, ax=axs[0,1])
        fig.colorbar(splt3, ax=axs[0,2])
        fig.colorbar(splt4, ax=axs[1,0])
        fig.colorbar(splt5, ax=axs[1,1])
        fig.colorbar(splt6, ax=axs[1,2])
        fig.colorbar(splt7, ax=axs[0,3])
        fig.colorbar(splt8, ax=axs[1,3])

        self.logger.info(f"!!!!!!!!! Took {time.time() - tmp1} seconds to make scatter plots")
        tmp1 = time.time()

        if folder:
            filename = f"{folder}/plts/{time.time()}_plt.png" if not is_flipped else f"{folder}/plts/flipped_{time.time()}_plt.png"
            plt.savefig(filename)
            self.logger.info(f"!!!!!!!!! Took {time.time() - tmp1} seconds to save fig")
            tmp1 = time.time()
            plt.close(fig)
            self.logger.info(f"!!!!!!!!! Took {time.time() - tmp1} seconds to close fig")

        
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

        # compute kernel components
        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(self.all_coords, evaluated_coords)
        Kxx = self.kernel(self.all_coords)

        # compute noise and y
        Sigma = np.eye(len(evaluated_points))
        for i, p in enumerate(evaluated_points):
            if not p.valid:
                p.known_certainty = self.tuner.validity_failure_model(p.max_planning_time / 2)
                p.certainty = self.tuner.validity_failure_model(p.max_planning_time / 2)
            # Sigma[i][i] = (1 - p.known_certainty)*np.max(self.cov)
            Sigma[i][i] = (1 - p.known_certainty)

        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times = np.array([p.max_planning_time for p in evaluated_points]).reshape((n, 1))
        
        self.inv = np.linalg.inv(KXX + 1e-10*np.eye(n) + Sigma)
        self.est_path_scores = KxX @ self.inv @ y
        self.cov = Kxx - KxX @ self.inv @ KxX.T
        self.est_planning_times = KxX @ np.linalg.inv(KXX + 1e-10*np.eye(n)) @ y_times
        # self.est_planning_times = KxX @ self.inv @ y_times

        np.clip(self.cov, 0.0, None, self.cov)

    def get_preds(self, coords):
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))

        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(coords, evaluated_coords)
        Kxx = self.kernel(coords)

        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times = np.array([p.max_planning_time for p in evaluated_points]).reshape((n, 1))

        u = KxX @ self.inv @ y
        c = Kxx - KxX @ self.inv @ KxX.T
        t = KxX @ np.linalg.inv(KXX + 1e-10*np.eye(n)) @ y_times
        # t = KxX @ self.inv @ y_times
        return u,c,t

    def update_max_path_score(self, newest_score):
        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = newest_score
        elif newest_score > self.max_path_score:
            self.max_path_score = newest_score

    def _handle_validity_send_goal(self, context):        
        self.logger.info("Sending Goal")
        idx, point = self.select_next()
        if point is None:
            context.all_points_certain = True
            if context.pending_results == 0:
                self._handle_validity_finish(context)
            return

        # update if necessary
        predicted_planning_time = self.est_planning_times[idx][0]
        self.logger.info(f"The estimated planning time is: {predicted_planning_time}")
        while predicted_planning_time > point.max_planning_time * (ProblemPoints.PLANNING_MULTIPLIER + 1)/2:
            self.logger.info("---------- UPDATING PLANNING TIME YIPPEEE -------")
            point.max_planning_time *= ProblemPoints.PLANNING_MULTIPLIER

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
        tmp1 = time.time()

        context.pending_results -= 1
        self.num_iterations += 1
        
        point = self.points[idx]
        point.handle_evaluation_result(result, self.tuner)
        # self.logger.info(f"------- Evaluated idx {idx} ----------")
        # cost = point.cost(math.inf)
        # self.logger.info(f"Certainty is now {point.certainty}, Path Score is now {point.path_score}, Cost {cost}")

        self.update_max_path_score(point.path_score)

        self.update_map()

        tmp2 = time.time()
        self.logger.info(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!! Took {tmp2 - tmp1} seconds to update map and stuff")

        tmp1 = time.time()
        if self.SAVE_DATA:
            hts_grasp.save_grasp_message(context.folder, context.is_flipped)
            hts_grasp.save_grasp_validity(context.folder, self.num_iterations == 1, context.is_flipped)

            # show the updated map
            if context.plot:
                self.plot_grasps(context.folder, context.is_flipped)
        
        tmp2 = time.time()
        self.logger.info(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Took {tmp2 - tmp1} seconds to save data")
        
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

        if context.visualise:
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
            else:
                best_grasp = best_point.grasp
                if context.visualise:
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
    PLANNING_MULTIPLIER = 2
    
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
        
        self.valid: bool = False
        self.evaluated: bool = False
    
    def get_certainty(self):
        return self.certainty
    
    def update_certainty_upon_eval(self, time_model: PlanningTimeTuner, time_taken: float):
        if self.valid:
            time_model.add_planning_data(time_taken)
            self.certainty = 1.0
            self.known_certainty = 1.0
        else:
            self.known_certainty = time_model.validity_failure_model(self.max_planning_time)
            self.certainty = self.known_certainty
            self.max_planning_time *= ProblemPoints.PLANNING_MULTIPLIER # double the planning time next time
    
    def cost(self, max_path_score):
        return self.norm_path_score(max_path_score)*self.grasp_score

    def norm_path_score(self, max_path_score):
        if not math.isinf(max_path_score) and not max_path_score == 0:
            return max(max_path_score - self.path_score, -max_path_score) / max_path_score
        else:
            return max(max_path_score - self.path_score, -max_path_score)
 
    def handle_evaluation_result(self, result: ComputeGraspValidity.Result, time_model: PlanningTimeTuner):
        self.evaluated = True
        self.valid = result.is_valid
        self.update_certainty_upon_eval(time_model, result.pickup_plan_time)
        self.path_score = result.score if result.is_valid else math.inf

class DualGraspSelectorGP():
    def __init__(self, group1: HTSGraspGroup, group2: HTSGraspGroup, context1, context2, final_context, logger, client, tuner: PlanningTimeTuner):
        self.selector1 = GraspSelectorGP(group1, logger, client, tuner)
        self.selector2 = GraspSelectorGP(group2, logger, client, tuner)

        self.selector1.start_timer()
        self.selector2.start_timer()

        self.context1 = context1
        self.context2 = context2
        self.final_context = final_context
        self.logger = logger
        self.client = client
        self.tuner = tuner

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

        # update if necessary
        predicted_planning_time = self.current_selector.est_planning_times[idx][0]
        self.logger.info(f"The estimated planning time is: {predicted_planning_time}")
        while predicted_planning_time > point.max_planning_time * (ProblemPoints.PLANNING_MULTIPLIER + 1)/2:
            self.logger.info("---------- UPDATING PLANNING TIME YIPPEEE -------")
            point.max_planning_time *= ProblemPoints.PLANNING_MULTIPLIER
        
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
        tmp1 = time.time()

        self.current_context.pending_results -= 1
        self.current_selector.num_iterations += 1
        
        point = self.current_selector.points[idx]
        point.handle_evaluation_result(result, self.tuner)
        self.current_selector.update_max_path_score(point.path_score)
        
        self.logger.info(f"------- Evaluated idx {idx} ----------")
        self.logger.info(f"Valid is {point.valid} Certainty is now {point.certainty}, Known Certainty is {point.known_certainty}, Path Score is now {point.path_score}, Norm Path Score is {point.norm_path_score(self.current_selector.max_path_score)} Cost is {point.cost(self.current_selector.max_path_score)}")
        self.logger.info(f"Max Path Score is {self.current_selector.max_path_score}")
        self.logger.info(f"Tuner is mean {self.tuner.planning_time_mean} var {self.tuner.planning_time_variance} data {self.tuner.planning_data}")

        self.current_selector.update_map()

        tmp2 = time.time()
        self.logger.info(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!! Took {tmp2 - tmp1} seconds to update map and stuff")
        tmp1 = time.time()

        if self.current_selector.SAVE_DATA:
            hts_grasp.save_grasp_message(self.current_context.folder, self.current_context.is_flipped)
            hts_grasp.save_grasp_validity(self.current_context.folder, self.current_selector.num_iterations == 1, self.current_context.is_flipped)

            # show the updated map
            # show the updated map
            if self.current_context.plot:
                self.current_selector.plot_grasps(self.current_context.folder, self.current_context.is_flipped)
        
        tmp2 = time.time()
        self.logger.info(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Took {tmp2 - tmp1} seconds to save data")

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

        if self.current_context.visualise:
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

            if self.current_context.visualise:
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
                final_response.grasp_pose = self.context1.response.grasp_pose
            elif (self.second_bestgrasp is not None):
                final_response.grasp_pose = self.context2.response.grasp_pose
            
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
    def __init__(self, goal_handle, grasp_group: HTSGraspGroup, folder, cloud, request, plot, visualise, is_flipped:bool = False):
        self.goal_handle = goal_handle
        self.hts_grasp_group = grasp_group
        self.pending_results = 0
        self.folder = folder
        self.cloud = cloud
        self.request = request
        self.response = None
        self.all_points_certain = False
        self.is_flipped = is_flipped
        self.plot = plot
        self.visualise = visualise