from __future__ import annotations
import numpy as np
import numpy.typing as npt
from sklearn.gaussian_process.kernels import RBF, Matern, ExpSineSquared, Kernel
from typing import Any, Sequence
import time
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib
import open3d as o3d
from abc import ABC, abstractmethod

try: # ros-dependent packages
    from .hts_grasps import HTSGraspGroup, HTSGrasp
    from .utils import ValidityContext, FakeLogger, display_grasps
    from .planning_time_models import PlanningTimeModel, LognormalPlanningTimeModel
    from .acquisition_functions import AcquisitionFunction, EpsilonGreedyUCB, SequentialAcquisition
    from .selection_points import GenericPoint, GraspPointBaseline, GraspPoint, CoordinatePoint
    from hts_msgs.action import ComputeGraspValidity, RequestGrasp
except ImportError as e:
    print(f"Received Import Error {e}, continuing")
    from hts_grasps import HTSGraspGroup, HTSGrasp
    from utils import ValidityContext, FakeLogger, display_grasps
    from planning_time_models import PlanningTimeModel, LognormalPlanningTimeModel
    from acquisition_functions import AcquisitionFunction, EpsilonGreedyUCB, SequentialAcquisition
    from selection_points import GenericPoint, GraspPointBaseline, GraspPoint, CoordinatePoint

# for speed purposes
matplotlib.use("agg")

class CompositeKernel(RBF):
    """A class which combines a distance and angle kernel together to create a 2D polar coordinate kernel"""

    def __init__(self, distance_kernel: Kernel, angle_kernel: Kernel):
        """
        Parameters:
            distance_kernel: the kernel used in the z (radial) direction
            angle_kernel: the kernel used in the theta (angular) direction
        """
        self.distance_kernel: Kernel = distance_kernel
        self.angle_kernel: Kernel = angle_kernel

    def __call__(self, X: Any, Y: Any | None=None, eval_gradient=False):
        # separate into radial and theta
        X_r: npt.NDArray[np.float64] = X[:, 0].reshape((-1, 1))
        X_th : npt.NDArray[np.float64]= X[:, 1].reshape((-1, 1))
        
        if Y is None:
            return self.distance_kernel(X_r, Y=None, eval_gradient=eval_gradient) * self.angle_kernel(X_th, Y=None, eval_gradient=eval_gradient)
        else:
            # separate into radial and theta
            Y_r: npt.NDArray[np.float64] = Y[:, 0].reshape((-1, 1))
            Y_th: npt.NDArray[np.float64] = Y[:, 1].reshape((-1, 1))
            return self.distance_kernel(X_r, Y=Y_r, eval_gradient=eval_gradient) * self.angle_kernel(X_th, Y=Y_th, eval_gradient=eval_gradient)

class GenericSelector(ABC):
    """A generic selector for choosing points"""

    def __init__(self, 
            collection: Any, 
            tuner: PlanningTimeModel,
            tuner_move: PlanningTimeModel,
            acquisition_function: AcquisitionFunction, 
            context: ValidityContext,
            **kwargs
            ):
        """
        Parameters:
            collection: a collection of objects which can be converted to points
            tuner: the planning time tuner which dictates how much time is given for evaluating each point
            acquisition_function: the function which controls selection of points to evaluate
            context: the context validity
        """
        
        self.num_iterations: int = 0 # how many iterations have elapsed
        self.context: ValidityContext = context # the validity context containing useful ros2 data
        self.acquisition_function: AcquisitionFunction = acquisition_function # the acquisition function
        self.tuner: PlanningTimeModel = tuner # the planning time model
        self.tuner_move: PlanningTimeModel = tuner_move

        # converts collection to points
        self.points: Sequence[GenericPoint] = type(self).to_points(collection) # the points coming from this collection

        # sets the initial planning time for each point
        for p in self.points:
            p.allocated_planning_time_pickup = tuner.get_initial_planning_time()
            p.allocated_planning_time_move = tuner_move.get_initial_planning_time()

        self.t0: float = 0.0 # the start time of the selector
        self.max_path_score: float = 0.0 # the current highest path score found

        self.N: int = len(self.points) # the number of points

        # coordinates used for update map
        self.all_z: npt.NDArray[np.float64] = np.array([p.z() for p in self.points]).reshape((-1, 1))
        self.all_th: npt.NDArray[np.float64] = np.array([p.theta() for p in self.points]).reshape((-1, 1))
        self.all_coords: npt.NDArray[np.float64] = np.hstack((self.all_z, self.all_th))

        # precompute x and y for plotting
        self.all_x: npt.NDArray[np.float64] = self.all_z*np.cos(self.all_th)
        self.all_y: npt.NDArray[np.float64] = self.all_z*np.sin(self.all_th)

        # precompute a mesh for map plotting
        self.x_mesh: npt.NDArray[np.float64]
        self.y_mesh: npt.NDArray[np.float64]
        self.x_mesh, self.y_mesh = np.meshgrid(
            np.linspace(np.min(self.all_x), np.max(self.all_x), 80), 
            np.linspace(np.min(self.all_y), np.max(self.all_y), 80)
        )
        self.x_mesh = np.array(self.x_mesh).reshape((-1, 1))
        self.y_mesh = np.array(self.y_mesh).reshape((-1, 1))

        self.z_mesh: npt.NDArray[np.float64] = np.sqrt(self.x_mesh**2 + self.y_mesh**2)
        self.th_mesh: npt.NDArray[np.float64] = np.arctan2(self.y_mesh, self.x_mesh)

        mesh_filter: npt.NDArray[np.bool_] = (self.z_mesh >= np.min(self.all_z)) & (self.z_mesh <= np.max(self.all_z))
        self.x_mesh = self.x_mesh[mesh_filter].reshape((-1, 1))
        self.y_mesh = self.y_mesh[mesh_filter].reshape((-1, 1))
        self.z_mesh = self.z_mesh[mesh_filter].reshape((-1, 1))
        self.th_mesh = self.th_mesh[mesh_filter].reshape((-1, 1))

        # parameters for the GP
        self.inv: npt.NDArray[np.float64] = np.array([])
        self.est_path_scores: npt.NDArray[np.float64] = np.zeros((self.N, 1))
        self.cov: npt.NDArray[np.float64] = np.eye(self.N)
        self.est_planning_times: npt.NDArray[np.float64] = np.ones((self.N, 1))*tuner.get_initial_planning_time()
        self.est_planning_times_move: npt.NDArray[np.float64] = np.ones((self.N, 1))*tuner_move.get_initial_planning_time()

        # start the timer
        self.start_timer()
    
    @classmethod
    @abstractmethod
    def to_points(cls, collection: Any) -> Sequence[GenericPoint]:
        """A generic method for converting a collection to a list of points"""
        pass

    def start_timer(self) -> None:
        """Starts the evalation timer of the selector"""
        self.t0 = time.time()
    
    @abstractmethod
    def select_next(self) -> tuple[int, GenericPoint | None]:      
        """
        Selects the next point to evaluate.
        
        Returns:
            a tuple of the index and point, or 0 and None if there is no next point to select
        """  
        pass

    @abstractmethod
    def plot_grasps(self, **kwargs) -> None:
        """Plots information about this grasps"""
        pass

    @abstractmethod
    def choose_best(self) -> tuple[GenericPoint | None, float]:
        """
        Chooses the best point this selector has evaluted.

        Returns:
            a tuple of the point and path cost, or None and 0 if no valid points have been found
        """
        pass

    def update_max_path_score(self, newest_score: float) -> None:
        """
        Updates the maximum path score that this selector has found.

        Parameters:
            newest_score: the path score of the most recently evaluated selector
        """

        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = newest_score
        elif newest_score > self.max_path_score:
            self.max_path_score = newest_score

    @abstractmethod
    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        """
        Sends a goal to evaluate a grasp.

        Parameters:
            callback_owner: which generic selector or dual selector should handle the response to this goal, or self if callback_owner is None
        """
        pass

    @abstractmethod
    def _handle_validity_response(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        """
        Handles the future response from a goal evaluation.

        Parameters:
            future: the future object
            idx: the index of the point being evaluated
            callback_owner: which generic selector or dual selector should handle the resolving of this future, or self if callback_owner is None
        """
        pass

    @abstractmethod
    def _handle_validity_result(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        """
        Handles the response from a goal evaluation after the future has been resolved.

        Parameters:
            future: the future object
            idx: the index of the point being evaluated
            callback_owner: which grasp selector or dual selector should handle any future actions, or self if callback_owner is None
        """
        pass

    @abstractmethod
    def _handle_validity_finish(self) -> None:
        """Handles once the grasp selection process has finished."""
        pass

class GPSelector(GenericSelector, ABC):
    """A generic selector using a gaussian process"""

    def __init__(self, 
        collection: Any, 
        tuner: PlanningTimeModel,
        tuner_move: PlanningTimeModel,
        acquisition_function: EpsilonGreedyUCB, 
        context: ValidityContext,
        length_scale_z: float = 0.03,
        matern_nu_z: float = 2.5,
        length_scale_th: float = 0.8,
        total_planning_time: float = 300,
        **kwargs
        ):
        """
        Parameters:
            collection: a collection of objects which can be converted to points
            tuner: the planning time tuner which dictates how much time is given for evaluating each point
            acquisition_function: the function which controls selection of points to evaluate
            context: the context validity
            length_scale_z: the length scale of the kernel in the z direction
            length_scale_th: the length scale of the kernel in the th direction
            matern_nu_z: the matern nu value in the z kernel
            total_planning_time: how much total planning time is allowed for this grasp selector
        """
        super().__init__(collection, tuner, tuner_move, acquisition_function, context, **kwargs)

        try: # ensure data types are correct
            assert context is not None
            assert type(acquisition_function) == EpsilonGreedyUCB
        except:
            raise AssertionError("Parameters of the GPSelector have the incorrect data types")

        self.logger: FakeLogger | Any = context.logger

        # create kernels
        self.dist_kernel = Matern(length_scale=length_scale_z, nu=matern_nu_z)
        self.angle_kernel = ExpSineSquared(length_scale=length_scale_th, periodicity=2*np.pi)
        self.kernel = CompositeKernel(self.dist_kernel, self.angle_kernel)

        self.dist_kernel_time = Matern(length_scale=length_scale_z/2, nu=0.5)
        self.angle_kernel_time = ExpSineSquared(length_scale=length_scale_th/2, periodicity=2*np.pi)
        self.kernel_time = CompositeKernel(self.dist_kernel_time, self.angle_kernel_time)

        self.total_planning_time: float = total_planning_time

    @abstractmethod
    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        """
        Sends a goal to evaluate a grasp.

        Parameters:
            callback_owner: which generic selector or dual selector should handle the response to this goal, or self if callback_owner is None
        """
        pass

    @abstractmethod
    def _handle_validity_response(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        """
        Handles the future response from a goal evaluation.

        Parameters:
            future: the future object
            idx: the index of the point being evaluated
            callback_owner: which generic selector or dual selector should handle the resolving of this future, or self if callback_owner is None
        """
        pass

    @abstractmethod
    def _handle_validity_result(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        """
        Handles the response from a goal evaluation after the future has been resolved.

        Parameters:
            future: the future object
            idx: the index of the point being evaluated
            callback_owner: which grasp selector or dual selector should handle any future actions, or self if callback_owner is None
        """
        pass

    @abstractmethod
    def _handle_validity_finish(self) -> None:
        """Handles once the grasp selection process has finished."""
        pass

    @classmethod
    @abstractmethod
    def to_points(cls, collection: Any) -> Sequence[GenericPoint]:
        """A generic method for converting a collection to a list of points"""
        pass

    def select_next(self) -> tuple[int, GenericPoint | None]:
        sampled_idx: int = self.acquisition_function.sample(
            self.est_path_scores.ravel(),
            np.diag(self.cov), 
            np.array([1 - p.certainty for p in self.points])
        )

        self.acquisition_function.time_step()

        if sampled_idx < 0 or sampled_idx >= len(self.points):
            return 0, None

        return sampled_idx, self.points[sampled_idx]

    def choose_best(self) -> tuple[GenericPoint | None, float]:
        best_cost: float = 0.0
        best_point: GenericPoint | None = None

        self.points: Sequence[GenericPoint] # type: ignore
        for p in self.points:
            if not p.evaluated:
                continue
            cost: float = p.cost(self.max_path_score)
            if cost > best_cost:
                best_cost = cost
                best_point = p

        return best_point, best_cost

    def update_map(self) -> None:
        """
        Updates the gaussian process parameters.
        """

        # construct training dataset from already sampled points
        evaluated_points: list[GenericPoint] = list(filter(lambda p: p.evaluated, self.points))
        n: int = len(evaluated_points)
        evaluated_z: npt.NDArray[np.float64] = np.array([p.z() for p in evaluated_points]).reshape((n, 1))
        evaluated_th: npt.NDArray[np.float64] = np.array([p.theta() for p in evaluated_points]).reshape((n, 1))
        evaluated_coords: npt.NDArray[np.float64] = np.hstack((evaluated_z, evaluated_th))

        # compute kernel components
        KXX: npt.NDArray[np.float64] = self.kernel(evaluated_coords)
        KxX: npt.NDArray[np.float64] = self.kernel(self.all_coords, evaluated_coords)
        Kxx: npt.NDArray[np.float64] = self.kernel(self.all_coords)

        KXX_time: npt.NDArray[np.float64] = self.kernel_time(evaluated_coords)
        KxX_time: npt.NDArray[np.float64] = self.kernel_time(self.all_coords, evaluated_coords)

        # compute noise and y
        Sigma: npt.NDArray[np.float64] = np.eye(len(evaluated_points))
        for i, p in enumerate(evaluated_points):
            if not p.valid:
                # we divide by the planner multiplier to get the certainty
                if p.invalidity_is_pickup:
                    p.certainty = self.tuner.validity_failure_model(p.allocated_planning_time_pickup / self.tuner.get_planning_multiplier())
                    self.logger.info(f"Point certainty is {p.certainty} due to pickup |  prev planning time {p.allocated_planning_time_pickup / self.tuner.get_planning_multiplier()} mean {self.tuner.planning_time_mean}")
                else:
                    p.certainty = self.tuner_move.validity_failure_model(p.allocated_planning_time_move / self.tuner_move.get_planning_multiplier())
                    self.logger.info(f"Point certainty is {p.certainty} due to move")
            Sigma[i][i] = (1 - p.get_certainty())

        # compute target y
        y: npt.NDArray[np.float64] = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times: npt.NDArray[np.float64] = np.array([p.allocated_planning_time_pickup for p in evaluated_points]).reshape((n, 1))
        y_times_move: npt.NDArray[np.float64] = np.array([p.allocated_planning_time_move for p in evaluated_points]).reshape((n, 1))
        
        # perform inversions TODO make this cholesky
        self.inv: npt.NDArray[np.float64] = np.linalg.inv(KXX + 1e-10*np.eye(n) + Sigma)
        self.est_path_scores = KxX @ self.inv @ y
        self.cov = Kxx - KxX @ self.inv @ KxX.T
        I = np.linalg.inv(KXX_time + 1e-10*np.eye(n))
        self.est_planning_times = KxX_time @ I @ y_times
        self.est_planning_times_move = KxX_time @ I @ y_times_move

        np.clip(self.cov, 0.0, None, self.cov)

    def get_preds(self, coords: npt.NDArray[np.float64]) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Predict the posteriors of a set of coordinates.

        Parameters:
            coords: the coordinates to evaluate
        
        Returns:
            a tuple of the posterior mean, covariance, and mean of the planning times
        """

        evaluated_points: list[GraspPoint] = list(filter(lambda p: p.evaluated, self.points))
        n: int = len(evaluated_points)
        evaluated_z: npt.NDArray[np.float64] = np.array([p.z() for p in evaluated_points]).reshape((n, 1))
        evaluated_th: npt.NDArray[np.float64] = np.array([p.theta() for p in evaluated_points]).reshape((n, 1))
        evaluated_coords: npt.NDArray[np.float64] = np.hstack((evaluated_z, evaluated_th))

        KXX: npt.NDArray[np.float64] = self.kernel(evaluated_coords)
        KxX: npt.NDArray[np.float64] = self.kernel(coords, evaluated_coords)
        Kxx: npt.NDArray[np.float64] = self.kernel(coords)

        y: npt.NDArray[np.float64] = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        y_times: npt.NDArray[np.float64] = np.array([p.allocated_planning_time_pickup for p in evaluated_points]).reshape((n, 1))
        y_times_move: npt.NDArray[np.float64] = np.array([p.allocated_planning_time_move for p in evaluated_points]).reshape((n, 1))

        KXX_time: npt.NDArray[np.float64] = self.kernel_time(evaluated_coords)
        KxX_time: npt.NDArray[np.float64] = self.kernel_time(coords, evaluated_coords)

        u: npt.NDArray[np.float64] = KxX @ self.inv @ y
        c: npt.NDArray[np.float64] = Kxx - KxX @ self.inv @ KxX.T
        I = np.linalg.inv(KXX_time + 1e-10*np.eye(n))
        t: npt.NDArray[np.float64] = KxX_time @ I @ y_times
        t_move: npt.NDArray[np.float64] = KxX_time @ I @ y_times_move
        return u,c,t,t_move

    def plot_grasps(self, **kwargs):
        # predict mean, cov and planning time mean for mesh
        mesh_mean: npt.NDArray[np.float64]
        mesh_cov: npt.NDArray[np.float64]
        mesh_times: npt.NDArray[np.float64]
        mesh_mean, mesh_cov, mesh_times, mesh_times_move = self.get_preds(np.hstack((self.z_mesh, self.th_mesh)))

        mesh_uncertanties: npt.NDArray[np.float64] = np.sqrt(np.diag(mesh_cov))

        # get the mean and uncertainty of the GP
        gp_mean: npt.NDArray[np.float64] = self.est_path_scores
        gp_uncertainties: npt.NDArray[np.float64] = np.sqrt(np.diag(self.cov))

        # whether a point has been evaluated or not
        evaluated: list[bool] = [x.evaluated for x in self.points]
        validity: list[int] = [(1 if x.valid else -1) if x.evaluated else 0 for x in self.points]

        # evaluated filter
        evaluated_points: list[GenericPoint] = list(filter(lambda p: p.evaluated, self.points))
        evaluated_z: npt.NDArray[np.float64] = np.array([p.z() for p in evaluated_points])
        evaluated_th: npt.NDArray[np.float64] = np.array([p.theta() for p in evaluated_points])
        
        evaluated_planning_pickup: list[float] = [p.allocated_planning_time_pickup for p in evaluated_points]
        evaluated_planning_move: list[float] = [p.allocated_planning_time_move for p in evaluated_points]

        evaluated_paths: list[float] = [p.norm_path_score(self.max_path_score) for p in evaluated_points]
        evaluated_uncertainty: list[float] = [1 - p.get_certainty() for p in evaluated_points]

        # weights 
        weights: npt.NDArray[np.float64] = (self.est_path_scores.ravel() + self.acquisition_function.kappa*np.sqrt(np.diag(self.cov)))
        weights = weights - np.min(weights)
        weights = weights * np.array([1 - p.certainty for p in self.points])

        clipped_planning_times: npt.NDArray[np.float64] = np.log2(np.clip(self.est_planning_times, self.tuner.get_initial_planning_time(), None))
        clipped_mesh_times: npt.NDArray[np.float64] = np.log2(np.clip(mesh_times, self.tuner.get_initial_planning_time(), None))

        fig, axs = plt.subplots(3, 4, figsize=(30, 24), subplot_kw={'projection': 'polar'})
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
            evaluated_th, evaluated_z,
            c=evaluated_planning_pickup, cmap=cm.RdYlGn_r,marker="o"
            )

        splt9 = axs[2, 0].scatter(
            self.th_mesh, self.z_mesh,
            c=mesh_times, cmap=cm.RdYlGn_r,marker=","
            )
        
        splt10 = axs[2, 1].scatter(
            self.th_mesh, self.z_mesh,
            c=mesh_times_move, cmap=cm.RdYlGn_r,marker=","
            )
        
        splt11 = axs[2, 3].scatter(
            evaluated_th, evaluated_z,
            c=evaluated_planning_move, cmap=cm.RdYlGn_r,marker="o"
            )
        
        axs[1, 0].scatter(
            evaluated_th, evaluated_z, vmin=np.min(mesh_mean), vmax=np.max(mesh_mean),
            c=evaluated_paths, cmap=cm.RdYlGn, marker="o", linewidths=0.3, edgecolors="black"
            )
        axs[1, 1].scatter(
            evaluated_th, evaluated_z, vmin=0.0, vmax=1.0,
            c=evaluated_uncertainty, cmap=cm.RdYlGn_r, marker="o", linewidths=0.3, edgecolors="black"
            )
        # axs[1, 3].scatter(
        #     self.all_th, self.all_z,
        #     c=clipped_planning_times - np.log2(self.tuner.get_initial_planning_time()), cmap=cm.RdYlGn_r, marker="o", linewidths=0.3, edgecolors="black"
        #     )

        axs[0, 0].set_title("Predicted Path Score (Normalised)")
        axs[0, 1].set_title("Path Score Uncertainty")
        axs[0, 2].set_title("Grasp Validity")

        axs[1, 0].set_title("GP Map of Pred. Path Score (Norm.)")
        axs[1, 1].set_title("GP Covariance Map")
        axs[1, 2].set_title("Evaluated Grasps")

        axs[0, 3].set_title("Sampling Weights")
        # axs[1, 3].set_title("Planning Time Multiplier")

        axs[2, 0].set_title("Estimated Planning Times Pickup")
        axs[2, 1].set_title("Estimated Planning Times Move")

        axs[1, 3].set_title("Actual Planning Times Pickup")
        axs[2, 3].set_title("Actual Planning Times Move")

        axs[0, 0].set_axisbelow(True)
        axs[0, 1].set_axisbelow(True)
        axs[0, 2].set_axisbelow(True)
        axs[0, 3].set_axisbelow(True)
        axs[1, 2].set_axisbelow(True)

        fig.colorbar(splt1, ax=axs[0,0])
        fig.colorbar(splt2, ax=axs[0,1])
        fig.colorbar(splt3, ax=axs[0,2])
        fig.colorbar(splt4, ax=axs[1,0])
        fig.colorbar(splt5, ax=axs[1,1])
        fig.colorbar(splt6, ax=axs[1,2])
        fig.colorbar(splt7, ax=axs[0,3])
        fig.colorbar(splt8, ax=axs[1,3])
        fig.colorbar(splt9, ax=axs[2,0])
        fig.colorbar(splt10, ax=axs[2,1])
        fig.colorbar(splt11, ax=axs[2,3])

        if self.context.folder:
            filename: str = f"{self.context.folder}/plts/{time.time()}_plt.png" if not self.context.is_flipped else f"{self.context.folder}/plts/flipped_{time.time()}_plt.png"
            plt.savefig(filename)
            plt.close(fig)

class GPGraspSelector(GPSelector):
    """A grasp selector for choosing a grasp using a gaussian process"""

    def __init__(self, 
        collection: HTSGraspGroup, 
        tuner: PlanningTimeModel, 
        tuner_move: PlanningTimeModel, 
        acquisition_function: EpsilonGreedyUCB, 
        context: ValidityContext,
        length_scale_z: float = 0.03,
        matern_nu_z: float = 2.5,
        length_scale_th: float = 0.8,
        total_planning_time: float = 300,
        **kwargs
        ):
        """
        Parameters:
            collection: a collection of objects which can be converted to points
            tuner: the planning time tuner which dictates how much time is given for evaluating each point
            acquisition_function: the function which controls selection of points to evaluate
            context: the context validity
            length_scale_z: the length scale of the kernel in the z direction
            length_scale_th: the length scale of the kernel in the th direction
            matern_nu_z: the matern nu value in the z kernel
            total_planning_time: how much total planning time is allowed for this grasp selector
        """

        super().__init__(collection, tuner, tuner_move, acquisition_function, context, length_scale_z=length_scale_z, matern_nu_z=matern_nu_z, length_scale_th=length_scale_th, total_planning_time=total_planning_time, **kwargs)

        try: # ensure data types are correct
            assert context is not None
            assert type(acquisition_function) == EpsilonGreedyUCB
            assert type(collection) == HTSGraspGroup
        except:
            raise AssertionError("Parameters of the GPGraspSelector have the incorrect data types")

        # type forwarding
        self.context: ValidityContext = context
        self.grasps: HTSGraspGroup = collection
        self.acquisition_function: EpsilonGreedyUCB = acquisition_function # type: ignore
        self.points: Sequence[GraspPoint] = self.points # type: ignore

    @classmethod
    def to_points(cls, collection: HTSGraspGroup) -> Sequence[GraspPoint]:
        return [GraspPoint(g) for g in collection.get_grasps()]

    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        # choose a point
        idx: int
        point: GenericPoint | None
        idx, point = self.select_next()

        if point is None: # then we must have gone through all possible points
            self.context.all_points_certain = True

            if self.context.pending_results == 0: # nothing is pending, finish immediately
                self._handle_validity_finish()

            return # don't send any more goals
        
        # make sure it is a grasp point
        try:
            assert isinstance(point, GraspPoint)
        except AssertionError:
            raise AssertionError("Sampled Point is not of type GraspPoint")

        # if we haven't evaluated the point, we can downgrade the planning time below the base planning time
        # give us whatever value gives us a 95% CI exp(mean + 1.96*std)
        if not point.evaluated and self.tuner.planning_data.shape[0] > 0:
            point.allocated_planning_time = math.exp(self.tuner.planning_time_mean + 0*math.sqrt(self.tuner.planning_time_variance))
            self.logger.info(f"We have a new point, so we reset it back to the 95% pickup CI of {point.allocated_planning_time}")
        if (not point.evaluated or point.invalidity_is_pickup) and self.tuner_move.planning_data.shape[0] > 0:
            point.allocated_planning_time_move = math.exp(self.tuner_move.planning_time_mean + 0*math.sqrt(self.tuner_move.planning_time_variance))
            self.logger.info(f"We have a new point, so we reset it back to the 95% move CI of {point.allocated_planning_time_move}")

        # fast forward planning time if necessary (if we are more than half way to the next planning time)
        predicted_planning_time_pickup: float = self.est_planning_times.ravel()[idx]
        predicted_planning_time_move: float = self.est_planning_times_move.ravel()[idx]
        self.logger.info(f"The estimated pickup planning time is: {predicted_planning_time_pickup}")
        
        while predicted_planning_time_pickup > point.allocated_planning_time_pickup * (self.tuner.get_planning_multiplier() + 1)/2:
            point.allocated_planning_time_pickup *= self.tuner.get_planning_multiplier()

        self.logger.info(f"The estimated move planning time is: {predicted_planning_time_move}")
        while predicted_planning_time_move > point.allocated_planning_time_move * (self.tuner_move.get_planning_multiplier() + 1)/2:
            point.allocated_planning_time_move *= self.tuner_move.get_planning_multiplier()

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
        # planning_time_pickup = point.allocated_planning_time_pickup if len(self.tuner.planning_data) != 0 else self.tuner.get_exploration_planning_time()
        # planning_time_move = point.allocated_planning_time_move if len(self.tuner_move.planning_data) != 0 else self.tuner_move.get_exploration_planning_time()
        planning_time_pickup = point.allocated_planning_time_pickup
        planning_time_move = point.allocated_planning_time_move
        self.logger.info(f"Use planning time for pickup {planning_time_pickup} and move {planning_time_move}")

        send_goal_future: Any = self.context.client.send_goal_async(grasp.evaluation_request_goal(
            self.context.request, planning_time_pickup, planning_time_move
            ))
        send_goal_future.add_done_callback(lambda f: (self if callback_owner is None else callback_owner)._handle_validity_response(f, idx))
            
    def _handle_validity_response(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None):
        goal_handle: Any = future.result()
        
        if not goal_handle.accepted: # if goal was rejected
            self.logger.warn("Goal Rejected")
            self.context.pending_results -= 1
            return
        
        # get the result of the future and attach a done callback
        result_future: Any = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f : (self if callback_owner is None else callback_owner)._handle_validity_result(f, idx))

    def _handle_validity_result(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None):
        result: Any = future.result().result

        # recover the grasp from the index
        hts_grasp: HTSGrasp = self.grasps.get_grasps()[idx]
        hts_grasp.process_result(result)
        hts_grasp.end_timer()

        # deregister the pending result and increment the number of iterations
        self.context.pending_results -= 1
        self.num_iterations += 1
        
        # recover the point from the index
        point: GraspPoint = self.points[idx]
        point.handle_evaluation_result(result, self.tuner, self.tuner_move+)
        point.grasp.process_result(result)
        point.grasp.end_timer()

        pickup_time_taken = result.pickup_plan_time
        move_time_taken = result.move_refine_time

        self.logger.info(f"Pickup time is {pickup_time_taken}, Move time is {move_time_taken}")
        self.logger.info(f"Adjusted pickup mean to {self.tuner.planning_time_mean} var {self.tuner.planning_time_variance}, Move time is {self.tuner_move.planning_time_mean} var {self.tuner_move.planning_time_variance}")
        self.logger.info(f"Mean data: {self.tuner.planning_data} Move data: {self.tuner_move.planning_data}")
        
        # self.logger.info(f"Pickup time is {pickup_time_taken}, Move time is {move_time_taken}")
        if result.is_valid: # we update both
            self.logger.info("Result is valid")
            self.certainty = 1.0
        elif move_time_taken == 0: # it is a pickup failure
            self.logger.info("Result is not valid because of a pickup failure")
        else: # it is a move failure
            self.logger.info("Result is not valid because of a move failure")

        # update the max score found 
        self.update_max_path_score(point.path_score)

        # update the GP map
        self.update_map()

        # save any data
        if self.context.save_data:
            hts_grasp.save_grasp_message(self.context.folder, self.context.is_flipped)
            hts_grasp.save_grasp_evaluations(self.context.folder, self.num_iterations == 1, self.context.is_flipped)

            # show the updated map
            if self.context.plot:
                self.plot_grasps()

        # determine whether the end conditions have been met
        t1: float = time.time()
        end_conditions_met: bool = (t1 - self.t0) > self.total_planning_time or self.context.all_points_certain

        if self.context.pending_results == 0 and end_conditions_met: # finish the selection process
            (self if callback_owner is None else callback_owner)._handle_validity_finish()
        elif not end_conditions_met: # run another iteration
            (self if callback_owner is None else callback_owner)._handle_validity_send_goal()
        else: # wait for pending results
            self.logger.info("Waiting for others to finish...")

    def _handle_validity_finish(self) -> None:
        self.plot_grasps()

        # request and responses of the goal planner
        request: RequestGrasp.Request = self.context.goal_handle.request
        feedback: RequestGrasp.Feedback = RequestGrasp.Feedback()
        response: RequestGrasp.Result = RequestGrasp.Result()

        hts_grasp_group: HTSGraspGroup = self.context.hts_grasp_group
        folder: str = self.context.folder
        cloud: o3d.cuda.pybind.geometry.PointCloud | None = self.context.cloud

        # publish some feedback
        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        self.context.goal_handle.publish_feedback(feedback)

        # save data
        if self.context.save_data:
            hts_grasp_group.save_metrics(folder, self.context.is_flipped)

        # visualise data
        if self.context.visualise and cloud is not None:
            hts_grasp_group.visualise(cloud, origin_position=[request.x, request.y, request.z], description="Grasp Scores")
            hts_grasp_group.filter_grasp_group(lambda x: not x.evaluated()).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Non Evaluated Grasps")
            hts_grasp_group.filter_grasp_group(HTSGrasp.is_valid).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Valid Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.pickup_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Pickup Failed Scores")
            hts_grasp_group.filter_grasp_group(HTSGrasp.move_failed).visualise(cloud, origin_position=[request.x, request.y, request.z], description="Move Failed Scores")

        # if a valid grasp was found, populate the response field of the context
        if hts_grasp_group.num_valid():
            self.logger.info("Found the best grasp")

            # get the best grasp
            best_point: GenericPoint | None
            best_point, _ = self.choose_best()

            if best_point is None: # this should not happen
                self.logger.info("No valid grasps found")
                response.success = False
                self.context.response = response
            else: # display and return the best grasp
                # make sure it is a grasp point
                try:
                    assert isinstance(best_point, GraspPoint)
                except AssertionError:
                    raise AssertionError("Best Point is not of type GraspPoint")

                best_grasp: HTSGrasp = best_point.grasp
                if self.context.visualise and cloud is not None:
                    display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

                response.grasp_pose = best_grasp.get_pose()
                self.logger.info("--> " + str(response.grasp_pose))
                response.success = True
                self.context.response = response
        else: # no valid grasps found
            self.logger.info("No valid grasps found")
            response.success = False
            self.context.response = response

class GPPointSelector(GPSelector):
    def __init__(self, 
            collection: list[CoordinatePoint], 
            tuner: PlanningTimeModel, 
            tuner_move: PlanningTimeModel, 
            acquisition_function: EpsilonGreedyUCB, 
            context: ValidityContext,
            length_scale_z: float = 0.03,
            matern_nu_z: float = 2.5,
            length_scale_th: float = 0.8,
            total_planning_time: float = 300,
            **kwargs
            ):
        """
        Parameters:
            collection: a collection of objects which can be converted to points
            tuner: the planning time tuner which dictates how much time is given for evaluating each point
            acquisition_function: the function which controls selection of points to evaluate
            context: the context validity
            length_scale_z: the length scale of the kernel in the z direction
            length_scale_th: the length scale of the kernel in the th direction
            matern_nu_z: the matern nu value in the z kernel
            total_planning_time: how much total planning time is allowed for this grasp selector
        """
        super().__init__(collection, tuner, tuner_move, acquisition_function, context, length_scale_z=length_scale_z, matern_nu_z=matern_nu_z, length_scale_th=length_scale_th, total_planning_time=total_planning_time, **kwargs)

        try: # ensure data types are correct
            assert context is not None
            assert type(acquisition_function) == EpsilonGreedyUCB
            assert type(collection) == list
            assert (not len(collection)) or type(collection[0]) == CoordinatePoint
        except:
            raise AssertionError("Parameters of the GPPointSelector have the incorrect data types")

        # type forwarding
        self.context: ValidityContext = context
        self.acquisition_function: EpsilonGreedyUCB = acquisition_function # type: ignore
        self.points: Sequence[CoordinatePoint] = self.points # type: ignore

    @classmethod
    def to_points(cls, collection: list[CoordinatePoint]) -> Sequence[CoordinatePoint]:
        return collection

    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        # choose a point
        idx: int
        point: GenericPoint | None
        idx, point = self.select_next()

        if point is None:
            # then we must have gone through all possible points
            self.context.all_points_certain = True

            if self.context.pending_results == 0: # nothing is pending, finish immediately
                self._handle_validity_finish()
            return # don't send any more goals

        # make sure it is a grasp point
        try:
            assert isinstance(point, CoordinatePoint)
        except AssertionError:
            raise AssertionError("Sampled Point is not of type CoordinatePoint")

        # fast forward planning time if necessary (if we are more than half way to the next planning time)
        predicted_planning_time: float = self.est_planning_times.ravel()[idx]
        self.logger.info(f"The estimated planning time is: {predicted_planning_time}")
        while predicted_planning_time > point.allocated_planning_time_pickup * (self.tuner.get_planning_multiplier() + 1)/2:
            point.allocated_planning_time_pickup *= self.tuner.get_planning_multiplier()

        if point.evaluated:
            self.logger.info(f"Candidate Point (Re-eval) {idx + 1}/{len(self.points)}")
        else:
            self.logger.info(f"Candidate Point {idx + 1}/{len(self.points)}")
        
        self.context.pending_results += 1 # register another pending result

        (self if callback_owner is None else callback_owner)._handle_validity_response(None, idx)
            
    def _handle_validity_response(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        (self if callback_owner is None else callback_owner)._handle_validity_result(None, idx)

    def _handle_validity_result(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        # deregister the pending result and increment the number of iterations
        self.context.pending_results -= 1
        self.num_iterations += 1

        # make sure it is a grasp point
        try:
            assert isinstance(self.points[idx], CoordinatePoint)
        except AssertionError:
            raise AssertionError("Result Point is not of type CoordinatePoint")

        # recover the point from the index
        point: CoordinatePoint = self.points[idx]
        result: float = point.known_path_score
        point.handle_evaluation_result(result, self.tuner)

        # update the max score found 
        self.update_max_path_score(point.path_score)

        # update the GP map
        self.update_map()

        self.plot_grasps()

        t1: float = time.time()
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

class SequentialGraspSelector(GenericSelector):
    def __init__(self, 
                 collection: HTSGraspGroup, 
                 tuner: PlanningTimeModel, 
                 tuner_move: PlanningTimeModel, 
                 acquisition_function: SequentialAcquisition, 
                 context: ValidityContext,
                 **kwargs
                 ):
        super().__init__(collection, tuner, tuner_move, acquisition_function, context, **kwargs)

        assert context is not None
        self.logger: FakeLogger | Any = context.logger

        # type forwarding
        self.context: ValidityContext = context
        self.grasps: HTSGraspGroup = collection
        self.acquisition_function: SequentialAcquisition = acquisition_function # type: ignore
        # self.points: Sequence[GraspPoint] = self.points # type: ignore

    @classmethod
    def to_points(cls, collection: HTSGraspGroup) -> Sequence[GraspPointBaseline]:
        return [GraspPointBaseline(g) for g in collection.get_grasps()]

    def select_next(self) -> tuple[int, GraspPointBaseline | None]:
        sampled_idx: int = self.acquisition_function.sample()
        self.acquisition_function.time_step()

        if sampled_idx < 0 or sampled_idx >= len(self.points):
            return 0, None

        return sampled_idx, self.points[sampled_idx]

    def choose_best(self) -> tuple[GraspPointBaseline | None, float]:
        best_cost: float = 0.0
        best_point: GraspPointBaseline | None = None

        self.points: Sequence[GraspPointBaseline] # type: ignore
        for p in self.points:
            if not p.evaluated:
                continue
            cost: float = p.cost(self.max_path_score)
            if cost > best_cost:
                best_cost = cost
                best_point = p

        return best_point, best_cost

    def _handle_validity_send_goal(self, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        # choose a point
        idx: int
        point: GraspPointBaseline | None
        idx, point = self.select_next()

        if point is None:
            # then we must have gone through all possible points
            self.context.all_points_certain = True

            if self.context.pending_results == 0: # nothing is pending, finish immediately
                self._handle_validity_finish()
            return # don't send any more goals

        # make sure it is a grasp point
        try:
            assert isinstance(point, GraspPointBaseline)
        except AssertionError:
            raise AssertionError("Sampled Point is not of type GraspPointBaseline")

        # fast forward planning time if necessary (if we are more than half way to the next planning time)
        point.allocated_planning_time_pickup = self.tuner.get_initial_planning_time()
        point.allocated_planning_time_move = self.tuner_move.get_initial_planning_time()

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
        # planning_time_pickup = point.allocated_planning_time_pickup if len(self.tuner.planning_data) != 0 else self.tuner.get_exploration_planning_time()
        # planning_time_move = point.allocated_planning_time_move if len(self.tuner_move.planning_data) != 0 else self.tuner_move.get_exploration_planning_time()
        planning_time_pickup = point.allocated_planning_time_pickup
        planning_time_move = point.allocated_planning_time_move

        self.logger.info(f"Use planning time for pickup {planning_time_pickup} and move {planning_time_move}")
        
        send_goal_future: Any = self.context.client.send_goal_async(grasp.evaluation_request_goal(
            self.context.request, planning_time_pickup, planning_time_move
            ))
        send_goal_future.add_done_callback(lambda f: (self if callback_owner is None else callback_owner)._handle_validity_response(f, idx))
            
    def _handle_validity_response(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        goal_handle: Any = future.result()
        
        if not goal_handle.accepted: # if goal was rejected
            self.logger.warn("Goal Rejected")
            self.context.pending_results -= 1
            return
        
        # get the result of the future and attach a done callback
        result_future: Any = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f : (self if callback_owner is None else callback_owner)._handle_validity_result(f, idx))

    def _handle_validity_result(self, future: Any, idx: int, callback_owner: GenericSelector | DualGraspSelector | None = None) -> None:
        result: Any = future.result().result

        # recover the grasp from the index
        hts_grasp : HTSGrasp = self.grasps.get_grasps()[idx]
        hts_grasp.process_result(result)
        hts_grasp.end_timer()

        # deregister the pending result and increment the number of iterations
        self.context.pending_results -= 1
        self.num_iterations += 1
        
        # recover the point from the index
        point: GraspPointBaseline = self.points[idx]
        point.handle_evaluation_result(result, self.tuner)
        point.grasp.process_result(result)
        point.grasp.end_timer()

        if self.context.save_data:
            hts_grasp.save_grasp_message(self.context.folder, self.context.is_flipped)
            hts_grasp.save_grasp_evaluations(self.context.folder, self.num_iterations == 1, self.context.is_flipped)

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

    def _handle_validity_finish(self) -> None:
        request: Any = self.context.goal_handle.request
        feedback: RequestGrasp.Feedback = RequestGrasp.Feedback()
        response: RequestGrasp.Result = RequestGrasp.Result()

        hts_grasp_group: HTSGraspGroup = self.context.hts_grasp_group
        folder: str = self.context.folder
        cloud: Any | None = self.context.cloud

        feedback.progress = f"Evaluated efficiency. {hts_grasp_group.num_valid()} valid grasps found."
        self.context.goal_handle.publish_feedback(feedback)

        # save data
        if self.context.save_data:
            hts_grasp_group.save_metrics(folder, self.context.is_flipped)
        self.plot_grasps()
        
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
            best_point: GraspPointBaseline | None
            best_point, _ = self.choose_best()

            if best_point is None: # this should not happen
                self.logger.info("No valid grasps found")
                response.success = False
                self.context.response = response
            else: # display and return the best grasp
                try:
                    assert type(best_point) == GraspPointBaseline
                except AssertionError:
                    raise AssertionError("Best Point is not a GraspPointBaseline")

                best_grasp: HTSGrasp = best_point.grasp
                if self.context.visualise:
                    display_grasps(best_grasp.single_grasp_group(), cloud, only_first=True, origin_position=[request.x, request.y, request.z], description="Best Grasp")

                response.grasp_pose = best_grasp.get_pose()
                self.logger.info("--> " + str(response.grasp_pose))
                response.success = True
                self.context.response = response
        else: # no valid grasps found
            self.logger.info("No valid grasps found")
            response.success = False
            self.context.response = response

    def plot_grasps(self, **kwargs) -> None:
        # whether a point has been evaluated or not
        evaluated: list[bool] = [x.evaluated for x in self.points]
        validity: list[int] = [(1 if x.valid else -1) if x.evaluated else 0 for x in self.points]

        # evaluated filter
        evaluated_points: list[GraspPointBaseline] = list(filter(lambda p: p.evaluated, self.points))
        evaluated_z: npt.NDArray[np.float64] = np.array([p.z() for p in evaluated_points])
        evaluated_th: npt.NDArray[np.float64] = np.array([p.theta() for p in evaluated_points])
        
        evaluated_paths: list[float] = [p.norm_path_score(self.max_path_score) for p in evaluated_points]
        evaluated_uncertainty: list[float] = [1 - p.get_certainty() for p in evaluated_points]

        fig, axs = plt.subplots(2, 2, figsize=(30, 12), subplot_kw={'projection': 'polar'})
        splt1 = axs[0, 0].scatter(
            self.all_th, self.all_z, vmin=-1.0, vmax=1.0,
            c=validity, cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
        )
        splt2 = axs[0, 1].scatter(
            evaluated_th, evaluated_z, vmin=0.0, vmax=1.0,
            cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
        )
        splt3 = axs[1, 0].scatter(
            evaluated_th, evaluated_z, vmin=-1.0, vmax=1.0,
            c=evaluated_paths, cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
        )
        splt4 = axs[1, 1].scatter(
            evaluated_th, evaluated_z, vmin=0.0, vmax=1.0,
            c=evaluated_uncertainty, cmap=cm.RdYlGn,marker="o", linewidths=0.3, edgecolors="black"
        )

        axs[0, 0].set_title("Grasp Validity")
        axs[0, 1].set_title("Evaluated Points")
        axs[1, 0].set_title("Evaluated Path Score (Normalised)")
        axs[1, 1].set_title("Evaluated Uncertainty")

        axs[0, 0].set_axisbelow(True)
        axs[0, 1].set_axisbelow(True)
        axs[1, 0].set_axisbelow(True)
        axs[1, 1].set_axisbelow(True)

        fig.colorbar(splt1, ax=axs[0,0])
        fig.colorbar(splt2, ax=axs[0,1])
        fig.colorbar(splt3, ax=axs[1,0])
        fig.colorbar(splt4, ax=axs[1,1])

        if self.context.folder:
            filename: str = f"{self.context.folder}/plts/{time.time()}_plt.png" if not self.context.is_flipped else f"{self.context.folder}/plts/flipped_{time.time()}_plt.png"
            plt.savefig(filename)
            plt.close(fig)

class DualGraspSelector(ABC):
    def __init__(self, selector1: GenericSelector, selector2: GenericSelector, final_context: ValidityContext, **kwargs):
        self.selector1: GenericSelector = selector1
        self.selector2: GenericSelector = selector2
        self.current_selector: GenericSelector = self.selector2
        self.final_context: ValidityContext = final_context

        self.first_terminated = False # if one selector has terminated

        self.selector1.start_timer()
        self.selector2.start_timer()

    def _handle_validity_send_goal(self) -> None:
        self.current_selector = self.selector2 if self.current_selector == self.selector1 else self.selector1
        self.current_selector._handle_validity_send_goal(callback_owner=self)

    def _handle_validity_response(self, future: Any, idx: int) -> None:
        self.current_selector._handle_validity_response(future, idx, callback_owner=self)

    def _handle_validity_result(self, future: Any, idx: int) -> None:
        self.current_selector._handle_validity_result(future, idx, callback_owner=self)

    def _handle_validity_finish(self) -> None:
        self.current_selector.context.logger.info(f"Handling Validity Finish for {self.current_selector == self.selector1}")
        self.current_selector._handle_validity_finish()

        response: Any | None = self.current_selector.context.response
        try:
            assert response is not None
        except AssertionError:
            raise AssertionError("Context Response has not been set")
        
        if not self.first_terminated:
            if response.success:
                best_point: None | GenericPoint
                best_point, best_cost = self.current_selector.choose_best()
                self.first_bestcost = best_cost
                assert best_point is not None
                self.first_bestgrasp = best_point
            self.first_terminated = True            
            self.current_selector = self.selector2 if self.current_selector == self.selector1 else self.selector1
            self._handle_validity_finish()
            return
        else:
            if response.success:
                best_point: None | GenericPoint
                best_point, best_cost = self.current_selector.choose_best()
                self.second_bestcost = best_cost
                assert best_point is not None
                self.second_bestgrasp = best_point

            try:
                assert self.selector1.context.response is not None and self.selector2.context.response is not None
            except AssertionError:
                raise AssertionError("Context Responses have not been set")

            response1_success: bool = self.selector1.context.response.success
            response2_success: bool = self.selector2.context.response.success
            self.selector1.context.logger.info(f"Success? {response1_success} {response2_success}")

            final_response: RequestGrasp.Result = RequestGrasp.Result()
            final_response.success = response1_success or response2_success
            if (response1_success and response2_success):
                if self.first_bestcost > self.second_bestcost:
                    final_response.grasp_pose = self.selector1.context.response.grasp_pose
                else:
                    final_response.grasp_pose = self.selector2.context.response.grasp_pose
            elif (response1_success):
                final_response.grasp_pose = self.selector1.context.response.grasp_pose
            elif (response2_success):
                final_response.grasp_pose = self.selector2.context.response.grasp_pose
            else:
                pass # the grasp pose is unset

            self.final_context.response = final_response

class DualGPGraspSelector(DualGraspSelector):
    def __init__(self, selector1: GPGraspSelector, selector2: GPGraspSelector, final_context: ValidityContext, **kwargs):
        super().__init__(selector1, selector2, final_context, **kwargs)

        self.first_bestgrasp: HTSGrasp | None = None
        self.first_bestcost: float = 0
        self.second_bestgrasp: HTSGrasp | None = None
        self.second_bestcost: float = 0

        self.selector1: GPGraspSelector # type: ignore
        self.selector2: GPGraspSelector # type: ignore
