from abc import ABC, abstractmethod
import math
from typing import Any

try: # ros-dependent packages
    from .hts_grasps import HTSGrasp
    from .planning_time_models import PlanningTimeModel
    from hts_msgs.action import ComputeGraspValidity
except ImportError as e:
    print(f"Received Import Error {e}, continuing")
    from hts_grasps import HTSGrasp
    from planning_time_models import PlanningTimeModel

class GenericPoint(ABC):
    """A generic point which represents a candidate for selection"""
    def __init__(self, _):
        """
        Parameters:
            _: ignored
        """
        self.certainty: float = 0 # how certain we are of our evaluation
        self._z: float = 0 # the vertical position of the grasp
        self._theta: float = 0 # the yaw of the grasp
        self.grasp_score: float = 0 # the grasp score
        self.path_score: float = math.inf # the computed path length, infinite if invalid
        self.allocated_planning_time_pickup: float = 0.03 # the planning time allocated for this grasp
        self.allocated_planning_time_move: float = 0.03
        self.valid: bool = False # whether this grasp is valid or not
        self.invalidity_is_pickup: bool = True 
        self.evaluated: bool = False # whether this grasp has been evaluated or not

    def get_certainty(self) -> float:
        """The certainty of this point"""
        return self.certainty

    def cost(self, max_path_score: float) -> float:
        """
        The cost of this point. Calculated as the normalised path score multiplied by the grasp score.

        Parameters:
            max_path_score: the largest path score found so far
        Returns:
            the point cost
        """
        return self.norm_path_score(max_path_score)*self.grasp_score
    
    def z(self) -> float:
        """The z coordinate of this point"""
        return self._z
    
    def theta(self) -> float:
        """The theta coordinate of this point"""
        return self._theta

    def norm_path_score(self, max_path_score: float) -> float:
        """
        Computes the normalised path score. Scales all valid path scores between 0 and 1 with the shortest path costs being closer to 1. 
        Invalid scores are assigned a path cost of -1, or if max_path_score is not finite and non-zero a 0.

        Parameters:
            max_path_score: the largest path score found so far
        Return:
            the normalised path score
        """
        if not math.isinf(max_path_score) and not max_path_score == 0:
            return max(max_path_score - self.path_score, -max_path_score) / max_path_score
        else:
            return max(max_path_score - self.path_score, -max_path_score)

    @abstractmethod
    def handle_evaluation_result(self, result: Any, time_model: PlanningTimeModel, time_model_move: PlanningTimeModel | None=None) -> None:
        """
        Processes the result of a grasp evaluation.

        Parameters:
            result: the grasp evaluation result
            time_model:  the planning time model this grasp evaluation interacts with
        """
        pass

    @abstractmethod
    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float) -> None:
        """
        Updates the certainty of the point and the time model upon an evaluation.

        Parameters:
            time_model: the planning time model this grasp evaluation interacts with
            time_taken: how long the planning took
        """
        pass

class GraspPoint(GenericPoint):
    """A grasp point when running the selection algorithm"""

    def __init__(self, grasp: HTSGrasp):
        """
        Parameters:
            grasp: the HTSGrasp of this point
        """
        super().__init__(grasp)

        # the HTSGrasp and z, theta
        self.grasp: HTSGrasp = grasp
        self._z: float = grasp.z
        self._theta: float = grasp.theta

        # retrieve the grasp score
        self.grasp_score: float = grasp.get_grasp_object().score

    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float) -> None:
        if self.valid:
            time_model.add_planning_data(time_taken)
            self.certainty = 1.0
        else:
            time_model.register_failed_plan()
            self.certainty = time_model.validity_failure_model(self.allocated_planning_time)
            self.allocated_planning_time *= time_model.get_planning_multiplier() # update planning time

    def handle_evaluation_result(self, result: ComputeGraspValidity.Result, time_model: PlanningTimeModel, time_model_move: PlanningTimeModel) -> None:
        self.evaluated = True
        self.valid = result.is_valid

        pickup_time_taken = result.pickup_plan_time
        move_time_taken = result.move_refine_time

        if result.is_valid: # we update both
            self.certainty = 1.0
            time_model.add_planning_data(pickup_time_taken)
            time_model_move.add_planning_data(move_time_taken)
        elif move_time_taken == 0: # it is a pickup failure
            time_model.register_failed_plan()
            self.certainty = time_model.validity_failure_model(pickup_time_taken)
            self.allocated_planning_time_pickup *= time_model.get_planning_multiplier() # update planning time
            self.invalidity_is_pickup = True
        else: # it is a move failure
            time_model.register_failed_plan()
            time_model.add_planning_data(pickup_time_taken)
            self.certainty = time_model_move.validity_failure_model(move_time_taken)
            self.allocated_planning_time_move *= time_model_move.get_planning_multiplier() # update planning time
            self.invalidity_is_pickup = False

        self.path_score = result.score if result.is_valid else math.inf

class GraspPointBaseline(GenericPoint):
    """A grasp point when running the baseline algorithm"""

    def __init__(self, grasp: HTSGrasp):
        super().__init__(grasp)

        # the HTSGrasp and z, theta
        self.grasp: HTSGrasp = grasp
        self._z: float = grasp.z
        self._theta: float = grasp.theta

        # retrieve the grasp score
        self.grasp_score: float = grasp.get_grasp_object().score

    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float) -> None:
        self.certainty = 1.0

    def handle_evaluation_result(self, result: ComputeGraspValidity.Result, time_model: PlanningTimeModel, time_model_move: PlanningTimeModel | None=None):
        self.evaluated = True
        self.valid = result.is_valid
        self.update_certainty_on_eval(time_model, float(result.pickup_plan_time))
        self.path_score = result.score if result.is_valid else math.inf

class CoordinatePoint(GenericPoint):
    """A point parametrised simply by its z and theta coordinates"""

    def __init__(self, coord: tuple[float, float]) -> None:
        super().__init__(coord)
        
        # the coordiantes
        self._z: float = coord[0]
        self._theta: float = coord[1]

        self.known_path_score: float = 0 # the known path score which will be returned

    def set_known_path_score(self, score: float) -> None:
        """
        Manually sets the path score value that this evaluation would return

        Parameters:
            score: the path score
        """
        self.known_path_score = score

    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float) -> None:
        self.certainty = 1.0

    def handle_evaluation_result(self, result: float, time_model: PlanningTimeModel, time_model_move: PlanningTimeModel | None=None):
        self.evaluated = True
        self.valid = not math.isinf(result)
        self.update_certainty_on_eval(time_model, 0.0)
        self.path_score = result if self.valid else math.inf
