from __future__ import annotations
import numpy as np
import scipy
from sklearn.gaussian_process.kernels import RBF, Matern, ExpSineSquared
from typing import Any, Sequence
import time
import random
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib
from abc import ABC, abstractmethod

try: # ros-dependent packages
    from .hts_grasps import HTSGraspGroup, HTSGrasp
    from .utils import ValidityContext, display_grasps
    from .planning_time_models import PlanningTimeModel, LognormalPlanningTimeModel
    from .acquisition_functions import AcquisitionFunction, EpsilonGreedyUCB, SequentialAcquisition
    from .selection_points import GenericPoint, GraspPointBaseline, GraspPoint, CoordinatePoint
    from hts_msgs.action import ComputeGraspValidity, RequestGrasp
except ImportError as e:
    print(f"Received Import Error {e}, continuing")
    from hts_grasps import HTSGraspGroup, HTSGrasp
    from utils import ValidityContext, display_grasps
    from planning_time_models import PlanningTimeModel, LognormalPlanningTimeModel
    from acquisition_functions import AcquisitionFunction, EpsilonGreedyUCB, SequentialAcquisition
    from selection_points import GenericPoint, GraspPointBaseline, GraspPoint, CoordinatePoint

matplotlib.use("agg")

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

class GenericSelector(ABC):
    def __init__(self, 
                 collection: Any, 
                 tuner: PlanningTimeModel, 
                 acquisition_function: AcquisitionFunction, 
                 context: ValidityContext,
                 **kwargs):
        self.num_iterations = 0 # how many iterations have elapsed
        self.context = context # the validity context containing useful ros2 data
        self.acquisition_function: AcquisitionFunction = acquisition_function # the acquisition function
        self.tuner: PlanningTimeModel = tuner # the planning time model
        self.points: Sequence[GenericPoint] = type(self).to_points(collection) # the points coming from this collection

        self.t0 = 0.0 # the start time of the selector
        self.max_path_score = 0.0 # the current highest path score found

        self.N = len(self.points) # the number of points

        # coordinates used for update map
        self.all_z = np.array([p.z() for p in self.points]).reshape((-1, 1))
        self.all_th = np.array([p.theta() for p in self.points]).reshape((-1, 1))
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
        self.est_planning_times = np.ones((self.N, 1))*tuner.get_initial_planning_time()

        self.start_timer()
    
    @classmethod
    @abstractmethod
    def to_points(cls, collection: Any) -> Sequence[GenericPoint]:
        pass

    def start_timer(self):
        self.t0 = time.time()
    
    @abstractmethod
    def select_next(self) -> tuple[int, GenericPoint | None]:        
        pass

    @abstractmethod
    def plot_grasps(self, **kwargs):
        pass

    @abstractmethod
    def choose_best(self) -> tuple[GenericPoint | None, float]:
        pass

    def update_max_path_score(self, newest_score: float):
        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = newest_score
        elif newest_score > self.max_path_score:
            self.max_path_score = newest_score

    @abstractmethod
    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None):
        pass

    @abstractmethod
    def _handle_validity_response(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None):
        pass

    @abstractmethod
    def _handle_validity_result(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None):
        pass

    @abstractmethod
    def _handle_validity_finish(self):
        pass

class GPGraspSelector(GenericSelector):
    def __init__(self, 
                 collection: HTSGraspGroup, 
                 tuner: PlanningTimeModel, 
                 acquisition_function: EpsilonGreedyUCB, 
                 context: ValidityContext,
                 length_scale_z: float = 0.03,
                 matern_nu_z: float = 2.5,
                 length_scale_th: float = 0.8,
                 total_planning_time: float = 300,
                 **kwargs
                 ):
        super().__init__(collection, tuner, acquisition_function, context, **kwargs)

        assert context is not None
        self.logger = context.logger

        # type forwarding
        self.context: ValidityContext = context
        self.grasps: HTSGraspGroup = collection
        self.acquisition_function: EpsilonGreedyUCB = acquisition_function # type: ignore
        # self.points: Sequence[GraspPoint] = self.points # type: ignore

        self.dist_kernel = Matern(length_scale=length_scale_z, nu=matern_nu_z)
        self.angle_kernel = ExpSineSquared(length_scale=length_scale_th, periodicity=2*np.pi)
        self.kernel = CompositeKernel(self.dist_kernel, self.angle_kernel)

        self.total_planning_time: float = total_planning_time

    @classmethod
    def to_points(cls, collection: HTSGraspGroup) -> Sequence[GraspPoint]:
        return [GraspPoint(g) for g in collection._grasps]

    def select_next(self) -> tuple[int, GraspPoint | None]:
        sampled_idx = self.acquisition_function.sample(
            self.est_path_scores.ravel(), 
            np.diag(self.cov), 
            np.array([1 - p.certainty for p in self.points])
        )

        self.acquisition_function.time_step()

        if sampled_idx < 0 or sampled_idx >= len(self.points):
            return 0, None

        return sampled_idx, self.points[sampled_idx]

    def choose_best(self) -> tuple[GraspPoint | None, float]:
        best_cost = 0.0
        best_point = None

        self.points: Sequence[GraspPoint] # type: ignore
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
        evaluated_z = np.array([p.z() for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta() for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))

        # compute kernel components
        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(self.all_coords, evaluated_coords)
        Kxx = self.kernel(self.all_coords)

        # compute noise and y
        Sigma = np.eye(len(evaluated_points))
        for i, p in enumerate(evaluated_points):
            if not p.valid:
                p.certainty = self.tuner.validity_failure_model(p.allocated_planning_time / 2)
            # Sigma[i][i] = (1 - p.known_certainty)*np.max(self.cov)
            Sigma[i][i] = (1 - p.get_certainty())

        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times = np.array([p.allocated_planning_time for p in evaluated_points]).reshape((n, 1))
        
        self.inv = np.linalg.inv(KXX + 1e-10*np.eye(n) + Sigma)
        self.est_path_scores = KxX @ self.inv @ y
        self.cov = Kxx - KxX @ self.inv @ KxX.T
        self.est_planning_times = KxX @ np.linalg.inv(KXX + 1e-10*np.eye(n)) @ y_times
        # self.est_planning_times = KxX @ self.inv @ y_times

        np.clip(self.cov, 0.0, None, self.cov)

    def get_preds(self, coords):
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z() for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta() for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))

        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(coords, evaluated_coords)
        Kxx = self.kernel(coords)

        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times = np.array([p.allocated_planning_time for p in evaluated_points]).reshape((n, 1))

        u = KxX @ self.inv @ y
        c = Kxx - KxX @ self.inv @ KxX.T
        t = KxX @ np.linalg.inv(KXX + 1e-10*np.eye(n)) @ y_times
        # t = KxX @ self.inv @ y_times
        return u,c,t

    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None):
        # choose a point
        idx, point = self.select_next()

        # make sure it is a grasp point
        assert isinstance(point, GraspPoint)

        if point is None:
            # then we must have gone through all possible points
            self.context.all_points_certain = True

            if self.context.pending_results == 0: # nothing is pending, finish immediately
                self._handle_validity_finish()
            return # don't send any more goals

        # fast forward planning time if necessary (if we are more than half way to the next planning time)
        predicted_planning_time: float = self.est_planning_times.ravel()[idx]
        self.logger.info(f"The estimated planning time is: {predicted_planning_time}")
        while predicted_planning_time > point.allocated_planning_time * (self.tuner.get_planning_multiplier() + 1)/2:
            point.allocated_planning_time *= self.tuner.get_planning_multiplier()

        # get the actual grasp
        grasp: HTSGrasp = point.grasp
        
        if point.evaluated:
            self.logger.info(f"Candidate Grasp (Re-eval) {idx + 1}/{len(self.points)}")
        else:
            self.logger.info(f"Candidate Grasp {idx + 1}/{len(self.points)}")
        
        # wait for the server
        self.context.client.wait_for_server()

        self.context.pending_results += 1 # register another pending result
        grasp.start_timer() # start recording the planning time for this grasp

        # send the goal and attach a done callback
        send_goal_future = self.context.client.send_goal_async(grasp.validity_request_goal(self.context.request, point.allocated_planning_time))
        send_goal_future.add_done_callback(lambda f: (self if callback_owner is None else callback_owner)._handle_validity_response(f, idx))
            
    def _handle_validity_response(self, future, idx, callback_owner: GenericSelector | DualGraspSelector | None = None):
        goal_handle = future.result()
        
        if not goal_handle.accepted: # if goal was rejected
            self.logger.warn("Goal Rejected")
            self.context.pending_results -= 1
            return
        
        # get the result of the future and attach a done callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f : (self if callback_owner is None else callback_owner)._handle_validity_result(f, idx))

    def _handle_validity_result(self, future, idx, callback_owner: GenericSelector | DualGraspSelector | None = None):
        
        result = future.result().result

        # recover the grasp from the index
        hts_grasp : HTSGrasp = self.grasps._grasps[idx]
        hts_grasp.process_result(result)
        hts_grasp.end_timer()

        # deregister the pending result and increment the number of iterations
        self.context.pending_results -= 1
        self.num_iterations += 1
        
        # recover the point from the index
        point = self.points[idx]
        point.handle_evaluation_result(result, self.tuner)
        point.grasp.process_result(result)
        point.grasp.end_timer()

        # update the max score found 
        self.update_max_path_score(point.path_score)

        # update the GP map
        self.update_map()

        if self.context.save_data:
            hts_grasp.save_grasp_message(self.context.folder, self.context.is_flipped)
            hts_grasp.save_grasp_validity(self.context.folder, self.num_iterations == 1, self.context.is_flipped)

            # show the updated map
            if self.context.plot:
                self.plot_grasps()

        t1 = time.time()
        end_conditions_met: bool = (t1 - self.t0) > self.total_planning_time or self.context.all_points_certain
        if self.context.pending_results == 0 and end_conditions_met: # finish the selection process
            (self if callback_owner is None else callback_owner)._handle_validity_finish()
        elif not end_conditions_met: # run another iteration
            (self if callback_owner is None else callback_owner)._handle_validity_send_goal()
        else: # wait for pending results
            self.logger.info("Waiting for others to finish...")

    def _handle_validity_finish(self):
        request = self.context.goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()

        hts_grasp_group = self.context.hts_grasp_group
        folder = self.context.folder
        cloud = self.context.cloud

        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        self.context.goal_handle.publish_feedback(feedback)

        # save data
        if self.context.save_data:
            hts_grasp_group.save_metrics(folder, self.context.is_flipped)

        # visualise data
        if self.context.visualise:
            hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
            hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
            hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        # if a valid grasp was found
        if hts_grasp_group.num_valid():
            self.logger.info("Found the best grasp")

            # get the best grasp
            best_point: GraspPoint | None
            best_point, _ = self.choose_best()

            if best_point is None: # this should not happen
                self.logger.info("No valid grasps found")
                response.success = False
                self.context.goal_handle.abort()
                self.context.response = response
            else: # display and return the best grasp
                best_grasp = best_point.grasp
                if self.context.visualise:
                    display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

                response.grasp_pose = best_grasp.get_pose()
                self.logger.info("--> " + str(response.grasp_pose))
                response.success = True
                self.context.goal_handle.succeed()
                self.context.response = response
        else: # no valid grasps found
            self.logger.info("No valid grasps found")
            response.success = False
            self.context.goal_handle.abort()
            self.context.response = response

    def plot_grasps(self, **kwargs):
        # predict mean and cov for mesh
        mesh_mean, mesh_cov, mesh_times = self.get_preds(np.hstack((self.z_mesh, self.th_mesh)))

        mesh_uncertanties = np.sqrt(np.diag(mesh_cov))

        # get the mean and uncertainty for 
        gp_mean = self.est_path_scores
        gp_uncertainties = np.sqrt(np.diag(self.cov))

        # whether a point has been evaluated or not
        evaluated = [x.evaluated for x in self.points]
        validity = [(1 if x.valid else -1) if x.evaluated else 0 for x in self.points]

        # evaluated filter
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        evaluated_z = np.array([p.z() for p in evaluated_points])
        evaluated_th = np.array([p.theta() for p in evaluated_points])
        
        evaluated_paths: list[float] = [p.norm_path_score(self.max_path_score) for p in evaluated_points]
        evaluated_uncertainty: list[float] = [1 - p.get_certainty() for p in evaluated_points]

        # weights 
        weights: np.ndarray = (self.est_path_scores.ravel() + self.acquisition_function.kappa*np.sqrt(np.diag(self.cov)))
        weights = weights - np.min(weights)
        weights = weights * np.array([1 - p.certainty for p in self.points])

        clipped_planning_times: np.ndarray = np.log2(np.clip(self.est_planning_times, self.tuner.get_initial_planning_time(), None))
        clipped_mesh_times: np.ndarray = np.log2(np.clip(mesh_times, self.tuner.get_initial_planning_time(), None))

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
            c=clipped_mesh_times - np.log2(self.tuner.get_initial_planning_time()), cmap=cm.RdYlGn_r,marker=","
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
            c=clipped_planning_times - np.log2(self.tuner.get_initial_planning_time()), cmap=cm.RdYlGn_r, marker="o", linewidths=0.3, edgecolors="black"
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

        if self.context.folder:
            filename = f"{self.context.folder}/plts/{time.time()}_plt.png" if not self.context.is_flipped else f"{self.context.folder}/plts/flipped_{time.time()}_plt.png"
            plt.savefig(filename)
            plt.close(fig)

class GPPointSelector(GenericSelector):
    def __init__(self, 
                 collection: list[CoordinatePoint], 
                 tuner: PlanningTimeModel, 
                 acquisition_function: EpsilonGreedyUCB, 
                 context: ValidityContext,
                 length_scale_z: float = 0.03,
                 matern_nu_z: float = 2.5,
                 length_scale_th: float = 0.8,
                 total_planning_time: float = 300,
                 **kwargs
                 ):
        super().__init__(collection, tuner, acquisition_function, context, **kwargs)

        assert context is not None
        self.logger = context.logger

        # type forwarding
        self.context: ValidityContext = context
        self.acquisition_function: EpsilonGreedyUCB = acquisition_function # type: ignore
        self.points: Sequence[CoordinatePoint] = self.points # type: ignore

        self.dist_kernel = Matern(length_scale=length_scale_z, nu=matern_nu_z)
        self.angle_kernel = ExpSineSquared(length_scale=length_scale_th, periodicity=2*np.pi)
        self.kernel = CompositeKernel(self.dist_kernel, self.angle_kernel)

        self.total_planning_time: float = total_planning_time

    @classmethod
    def to_points(cls, collection: list[CoordinatePoint]) -> Sequence[CoordinatePoint]:
        return collection

    def select_next(self) -> tuple[int, CoordinatePoint | None]:
        sampled_idx = self.acquisition_function.sample(
            self.est_path_scores.ravel(), 
            np.diag(self.cov), 
            np.array([1 - p.certainty for p in self.points])
        )

        self.acquisition_function.time_step()

        if sampled_idx < 0 or sampled_idx >= len(self.points):
            return 0, None

        return sampled_idx, self.points[sampled_idx]

    def choose_best(self) -> tuple[CoordinatePoint | None, float]:
        best_cost = 0.0
        best_point = None

        self.points: Sequence[CoordinatePoint] # type: ignore
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
        evaluated_z = np.array([p.z() for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta() for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))

        # compute kernel components
        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(self.all_coords, evaluated_coords)
        Kxx = self.kernel(self.all_coords)

        # compute noise and y
        Sigma = np.eye(len(evaluated_points))
        for i, p in enumerate(evaluated_points):
            # if not p.valid:
                # p.certainty = self.tuner.validity_failure_model(p.allocated_planning_time / 2)
            # Sigma[i][i] = (1 - p.known_certainty)*np.max(self.cov)
            Sigma[i][i] = (1 - 1)

        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times = np.array([p.allocated_planning_time for p in evaluated_points]).reshape((n, 1))
        
        self.inv = np.linalg.inv(KXX + 1e-10*np.eye(n) + Sigma)
        self.est_path_scores = KxX @ self.inv @ y
        self.cov = Kxx - KxX @ self.inv @ KxX.T
        self.est_planning_times = KxX @ np.linalg.inv(KXX + 1e-10*np.eye(n)) @ y_times
        # self.est_planning_times = KxX @ self.inv @ y_times

        np.clip(self.cov, 0.0, None, self.cov)

        if (np.any(np.isnan(self.est_path_scores))):
            print("OH ON")

    def get_preds(self, coords):
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z() for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta() for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))

        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(coords, evaluated_coords)
        Kxx = self.kernel(coords)

        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times = np.array([p.allocated_planning_time for p in evaluated_points]).reshape((n, 1))

        u = KxX @ self.inv @ y
        c = Kxx - KxX @ self.inv @ KxX.T
        t = KxX @ np.linalg.inv(KXX + 1e-10*np.eye(n)) @ y_times
        # t = KxX @ self.inv @ y_times
        return u,c,t

    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None):
        # choose a point
        idx, point = self.select_next()

        # make sure it is a grasp point
        assert isinstance(point, CoordinatePoint)

        if point is None:
            # then we must have gone through all possible points
            self.context.all_points_certain = True

            if self.context.pending_results == 0: # nothing is pending, finish immediately
                self._handle_validity_finish()
            return # don't send any more goals

        # fast forward planning time if necessary (if we are more than half way to the next planning time)
        predicted_planning_time: float = self.est_planning_times.ravel()[idx]
        self.logger.info(f"The estimated planning time is: {predicted_planning_time}")
        while predicted_planning_time > point.allocated_planning_time * (self.tuner.get_planning_multiplier() + 1)/2:
            point.allocated_planning_time *= self.tuner.get_planning_multiplier()

        if point.evaluated:
            self.logger.info(f"Candidate Point (Re-eval) {idx + 1}/{len(self.points)}")
        else:
            self.logger.info(f"Candidate Point {idx + 1}/{len(self.points)}")
        
        self.context.pending_results += 1 # register another pending result

        (self if callback_owner is None else callback_owner)._handle_validity_response(None, idx)
            
    def _handle_validity_response(self, future, idx, callback_owner: GenericSelector | DualGraspSelector | None = None):
        (self if callback_owner is None else callback_owner)._handle_validity_result(None, idx)

    def _handle_validity_result(self, future, idx, callback_owner: GenericSelector | DualGraspSelector | None = None):
        # deregister the pending result and increment the number of iterations
        self.context.pending_results -= 1
        self.num_iterations += 1

        # recover the point from the index
        self.points: Sequence[CoordinatePoint]
        point = self.points[idx]
        result = point.known_path_score
        point.handle_evaluation_result(result, self.tuner)

        # update the max score found 
        self.update_max_path_score(point.path_score)

        # update the GP map
        self.update_map()

        self.plot_grasps()

        t1 = time.time()
        end_conditions_met: bool = (t1 - self.t0) > self.total_planning_time or self.context.all_points_certain
        if self.context.pending_results == 0 and end_conditions_met: # finish the selection process
            (self if callback_owner is None else callback_owner)._handle_validity_finish()
        elif not end_conditions_met: # run another iteration
            (self if callback_owner is None else callback_owner)._handle_validity_send_goal()
        else: # wait for pending results
            self.logger.info("Waiting for others to finish...")

    def _handle_validity_finish(self):
        self.logger.info("Finished.")
        self.plot_grasps()

    def plot_grasps(self, **kwargs):
        # predict mean and cov for mesh
        mesh_mean, mesh_cov, mesh_times = self.get_preds(np.hstack((self.z_mesh, self.th_mesh)))

        mesh_uncertanties = np.sqrt(np.diag(mesh_cov))

        # get the mean and uncertainty for 
        gp_mean = self.est_path_scores
        gp_uncertainties = np.sqrt(np.diag(self.cov))

        # whether a point has been evaluated or not
        evaluated = [x.evaluated for x in self.points]
        validity = [(1 if x.valid else -1) if x.evaluated else 0 for x in self.points]

        # evaluated filter
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        evaluated_z = np.array([p.z() for p in evaluated_points])
        evaluated_th = np.array([p.theta() for p in evaluated_points])
        
        evaluated_paths: list[float] = [p.norm_path_score(self.max_path_score) for p in evaluated_points]
        evaluated_uncertainty: list[float] = [1 - p.get_certainty() for p in evaluated_points]

        # weights 
        weights: np.ndarray = (self.est_path_scores.ravel() + self.acquisition_function.kappa*np.sqrt(np.diag(self.cov)))
        weights = weights - np.min(weights)
        weights = weights * np.array([1 - p.certainty for p in self.points])

        clipped_planning_times: np.ndarray = np.log2(np.clip(self.est_planning_times, self.tuner.get_initial_planning_time(), None))
        clipped_mesh_times: np.ndarray = np.log2(np.clip(mesh_times, self.tuner.get_initial_planning_time(), None))

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
            c=clipped_mesh_times - np.log2(self.tuner.get_initial_planning_time()), cmap=cm.RdYlGn_r,marker=","
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
            c=clipped_planning_times - np.log2(self.tuner.get_initial_planning_time()), cmap=cm.RdYlGn_r, marker="o", linewidths=0.3, edgecolors="black"
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

        plt.show()

class SequentialGraspSelector(GenericSelector):
    def __init__(self, 
                 collection: HTSGraspGroup, 
                 tuner: PlanningTimeModel, 
                 acquisition_function: SequentialAcquisition, 
                 context: ValidityContext,
                 **kwargs
                 ):
        super().__init__(collection, tuner, acquisition_function, context, **kwargs)

        assert context is not None
        self.logger = context.logger

        # type forwarding
        self.context: ValidityContext = context
        self.grasps: HTSGraspGroup = collection
        self.acquisition_function: SequentialAcquisition = acquisition_function # type: ignore
        # self.points: Sequence[GraspPoint] = self.points # type: ignore

    @classmethod
    def to_points(cls, collection: HTSGraspGroup) -> Sequence[GraspPointBaseline]:
        return [GraspPointBaseline(g) for g in collection._grasps]

    def select_next(self) -> tuple[int, GraspPointBaseline | None]:
        sampled_idx = self.acquisition_function.sample()
        self.acquisition_function.time_step()

        if sampled_idx < 0 or sampled_idx >= len(self.points):
            return 0, None

        return sampled_idx, self.points[sampled_idx]

    def choose_best(self) -> tuple[GraspPointBaseline | None, float]:
        best_cost = 0.0
        best_point = None

        self.points: Sequence[GraspPointBaseline] # type: ignore
        for p in self.points:
            if not p.evaluated:
                continue
            cost = p.cost(self.max_path_score)
            if cost > best_cost:
                best_cost = cost
                best_point = p

        return best_point, best_cost

    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None):
        # choose a point
        idx, point = self.select_next()

        if point is None:
            # then we must have gone through all possible points
            self.context.all_points_certain = True

            if self.context.pending_results == 0: # nothing is pending, finish immediately
                self._handle_validity_finish()
            return # don't send any more goals

        # make sure it is a grasp point
        assert isinstance(point, GraspPointBaseline)

        # fast forward planning time if necessary (if we are more than half way to the next planning time)
        predicted_planning_time: float = self.est_planning_times.ravel()[idx]
        self.logger.info(f"The estimated planning time is: {predicted_planning_time}")
        while predicted_planning_time > point.allocated_planning_time * (self.tuner.get_planning_multiplier() + 1)/2:
            point.allocated_planning_time *= self.tuner.get_planning_multiplier()

        # get the actual grasp
        grasp: HTSGrasp = point.grasp
        
        if point.evaluated:
            self.logger.info(f"Candidate Grasp (Re-eval) {idx + 1}/{len(self.points)}")
        else:
            self.logger.info(f"Candidate Grasp {idx + 1}/{len(self.points)}")
        
        # wait for the server
        self.context.client.wait_for_server()

        self.context.pending_results += 1 # register another pending result
        grasp.start_timer() # start recording the planning time for this grasp

        # send the goal and attach a done callback
        send_goal_future = self.context.client.send_goal_async(grasp.validity_request_goal(self.context.request, point.allocated_planning_time))
        send_goal_future.add_done_callback(lambda f: (self if callback_owner is None else callback_owner)._handle_validity_response(f, idx))
            
    def _handle_validity_response(self, future, idx, callback_owner: GenericSelector | DualGraspSelector | None = None):
        goal_handle = future.result()
        
        if not goal_handle.accepted: # if goal was rejected
            self.logger.warn("Goal Rejected")
            self.context.pending_results -= 1
            return
        
        # get the result of the future and attach a done callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f : (self if callback_owner is None else callback_owner)._handle_validity_result(f, idx))

    def _handle_validity_result(self, future, idx, callback_owner: GenericSelector | DualGraspSelector | None = None):
        result = future.result().result

        # recover the grasp from the index
        hts_grasp : HTSGrasp = self.grasps._grasps[idx]
        hts_grasp.process_result(result)
        hts_grasp.end_timer()

        # deregister the pending result and increment the number of iterations
        self.context.pending_results -= 1
        self.num_iterations += 1
        
        # recover the point from the index
        point = self.points[idx]
        point.handle_evaluation_result(result, self.tuner)
        point.grasp.process_result(result)
        point.grasp.end_timer()

        if self.context.save_data:
            hts_grasp.save_grasp_message(self.context.folder, self.context.is_flipped)
            hts_grasp.save_grasp_validity(self.context.folder, self.num_iterations == 1, self.context.is_flipped)

            # show the updated map
            if self.context.plot:
                self.plot_grasps()

        end_conditions_met: bool = self.context.all_points_certain
        if self.context.pending_results == 0 and end_conditions_met: # finish the selection process
            (self if callback_owner is None else callback_owner)._handle_validity_finish()
        elif not end_conditions_met: # run another iteration
            (self if callback_owner is None else callback_owner)._handle_validity_send_goal()
        else: # wait for pending results
            self.logger.info("Waiting for others to finish...")

    def _handle_validity_finish(self):
        request = self.context.goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()

        hts_grasp_group = self.context.hts_grasp_group
        folder = self.context.folder
        cloud = self.context.cloud

        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        self.context.goal_handle.publish_feedback(feedback)

        # save data
        if self.context.save_data:
            hts_grasp_group.save_metrics(folder, self.context.is_flipped)

        # visualise data
        if self.context.visualise:
            hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
            hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
            hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        # if a valid grasp was found
        if hts_grasp_group.num_valid():
            self.logger.info("Found the best grasp")

            # get the best grasp
            best_point: GraspPoint | None
            best_point, _ = self.choose_best()

            if best_point is None: # this should not happen
                self.logger.info("No valid grasps found")
                response.success = False
                self.context.goal_handle.abort()
                self.context.response = response
            else: # display and return the best grasp
                best_grasp = best_point.grasp
                if self.context.visualise:
                    display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

                response.grasp_pose = best_grasp.get_pose()
                self.logger.info("--> " + str(response.grasp_pose))
                response.success = True
                self.context.goal_handle.succeed()
                self.context.response = response
        else: # no valid grasps found
            self.logger.info("No valid grasps found")
            response.success = False
            self.context.goal_handle.abort()
            self.context.response = response

    def plot_grasps(self, **kwargs):
        return

class DualGraspSelector(ABC):
    def __init__(self, selector1: GenericSelector, selector2: GenericSelector, final_context: ValidityContext, **kwargs):
        self.selector1: GenericSelector = selector1
        self.selector2: GenericSelector = selector2
        self.current_selector: GenericSelector = self.selector2
        self.final_context = final_context

        self.first_terminated = False # if one selector has terminated

        self.selector1.start_timer()
        self.selector2.start_timer()

    def _handle_validity_send_goal(self):
        self.current_selector = self.selector2 if self.current_selector == self.selector1 else self.selector1
        self.current_selector._handle_validity_send_goal(callback_owner=self)

    def _handle_validity_response(self, future, idx):
        self.current_selector._handle_validity_response(future, idx, callback_owner=self)

    def _handle_validity_result(self, future, idx):
        self.current_selector._handle_validity_result(future, idx, callback_owner=self)

    def _handle_validity_finish(self):
        self.current_selector.context.logger.info(f"Handling Validity Finish for {self.current_selector == self.selector1}")
        
        request = self.current_selector.context.goal_handle.request
        feedback = RequestGrasp.Feedback()
        response = RequestGrasp.Result()

        hts_grasp_group = self.current_selector.context.hts_grasp_group
        folder = self.current_selector.context.folder
        cloud = self.current_selector.context.cloud

        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        self.current_selector.context.goal_handle.publish_feedback(feedback)

        if self.current_selector.context.save_data:
            hts_grasp_group.save_metrics(folder, self.current_selector.context.is_flipped)

        if self.current_selector.context.visualise:
            hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
            hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
            hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        if hts_grasp_group.num_valid():
            self.current_selector.context.logger.info("Found the best grasp")

            best_point: None | GraspPoint
            best_point, best_cost = self.current_selector.choose_best()

            if best_point is None:
                self.current_selector.context.logger.info("No valid grasps found")
                response.success = False
            else:
                best_grasp = best_point.grasp
            
                if not self.first_terminated:
                    self.first_bestcost = best_cost
                    self.first_bestgrasp = best_grasp
                else:
                    self.second_bestcost = best_cost
                    self.second_bestgrasp = best_grasp

                if self.current_selector.context.visualise:
                    display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

                response.grasp_pose = best_grasp.get_pose()
                self.current_selector.context.logger.info("--> " + str(response.grasp_pose))
                response.success = True
        else:
            self.current_selector.context.logger.info("No valid grasps found")
            response.success = False

        self.current_selector.context.response = response

        if self.first_terminated:
            self.selector1.context.logger.info("Now the first terminated")
            self.selector1.context.logger.info(f"Success? {self.selector1.context.response.success} {self.selector2.context.response.success}")
            final_response = RequestGrasp.Result()
            final_response.success = self.selector1.context.response.success or self.selector2.context.response.success
            if (self.first_bestgrasp is not None and self.second_bestgrasp is not None):
                if self.first_bestcost > self.second_bestcost:
                    final_response.grasp_pose = self.first_bestgrasp.get_pose()
                else:
                    final_response.grasp_pose = self.second_bestgrasp.get_pose()
            elif (self.first_bestgrasp is not None):
                final_response.grasp_pose = self.selector1.context.response.grasp_pose
            elif (self.second_bestgrasp is not None):
                final_response.grasp_pose = self.selector2.context.response.grasp_pose
            
            if final_response.success:
                self.final_context.goal_handle.succeed()
            else:
                self.final_context.goal_handle.abort()
            self.final_context.response = final_response
        else:
            self.first_terminated = True
            self.current_selector = self.selector2 if self.current_selector == self.selector1 else self.selector1
            self._handle_validity_finish()

class DualGPGraspSelector(DualGraspSelector):
    def __init__(self, selector1: GPGraspSelector, selector2: GPGraspSelector, final_context: ValidityContext, **kwargs):
        super().__init__(selector1, selector2, final_context, **kwargs)

        self.first_bestgrasp: HTSGrasp | None = None
        self.first_bestcost: float = 0
        self.second_bestgrasp: HTSGrasp | None = None
        self.second_bestcost: float = 0

        self.selector1: GPGraspSelector # type: ignore
        self.selector2: GPGraspSelector # type: ignore
