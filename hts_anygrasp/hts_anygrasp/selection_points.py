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
    def __init__(self, _):
        self.certainty: float = 0 # how certain we are of our evaluation
        self._z: float = 0 # the vertical position of the grasp
        self._theta: float = 0 # the yaw of the grasp
        self.grasp_score: float = 0 # the grasp score
        self.path_score: float = math.inf # the computed path length
        self.allocated_planning_time: float = 0.03 # the planning time allocated for this grasp
        self.valid: bool = False # whether this grasp is valid or not
        self.evaluated: bool = False # whether this grasp has been evaluated or not

    def get_certainty(self):
        return self.certainty

    def cost(self, max_path_score: float):
        return self.norm_path_score(max_path_score)*self.grasp_score
    
    def z(self) -> float:
        return self._z
    
    def theta(self) -> float:
        return self._theta

    def norm_path_score(self, max_path_score: float) -> float:
        if not math.isinf(max_path_score) and not max_path_score == 0:
            return max(max_path_score - self.path_score, -max_path_score) / max_path_score
        else:
            return max(max_path_score - self.path_score, -max_path_score)

    @abstractmethod
    def handle_evaluation_result(self, result: Any, time_model: PlanningTimeModel):
        pass

    @abstractmethod
    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float):
        pass

class GraspPoint(GenericPoint):
    def __init__(self, grasp: HTSGrasp):
        super().__init__(grasp)

        self.grasp: HTSGrasp = grasp
        self._z: float = grasp.z
        self._theta: float = grasp.theta
        self.grasp_score: float = grasp.get_grasp_object().score

    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float):
        if self.valid:
            time_model.add_planning_data(time_taken)
            self.certainty = 1.0
        else:
            self.certainty = time_model.validity_failure_model(self.allocated_planning_time)
            self.allocated_planning_time *= time_model.get_planning_multiplier()

    def handle_evaluation_result(self, result: ComputeGraspValidity.Result, time_model: PlanningTimeModel):
        self.evaluated = True
        self.valid = result.is_valid
        self.update_certainty_on_eval(time_model, float(result.pickup_plan_time))
        self.path_score = result.score if result.is_valid else math.inf

class GraspPointBaseline(GenericPoint):
    def __init__(self, grasp: HTSGrasp):
        super().__init__(grasp)

        self.grasp: HTSGrasp = grasp
        self._z: float = grasp.z
        self._theta: float = grasp.theta
        self.grasp_score: float = grasp.get_grasp_object().score

    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float):
        self.certainty = 1.0

    def handle_evaluation_result(self, result: ComputeGraspValidity.Result, time_model: PlanningTimeModel):
        self.evaluated = True
        self.valid = result.is_valid
        self.update_certainty_on_eval(time_model, float(result.pickup_plan_time))
        self.path_score = result.score if result.is_valid else math.inf

class CoordinatePoint(GenericPoint):
    def __init__(self, coord: tuple[float, float]):
        super().__init__(coord)
        
        self._z: float = coord[0]
        self._theta: float = coord[1]
        self.known_path_score: float = 0

    def set_known_path_score(self, score: float):
        self.known_path_score = score

    def update_certainty_on_eval(self, time_model: PlanningTimeModel, time_taken: float):
        if self.valid:
            # time_model.add_planning_data(time_taken)
            self.certainty = 1.0
        else:
            self.certainty = 1.0
            # self.allocated_planning_time *= time_model.get_planning_multiplier()

    def handle_evaluation_result(self, result: float, time_model: PlanningTimeModel):
        self.evaluated = True
        self.valid = not math.isinf(result)
        self.update_certainty_on_eval(time_model, 0.0)
        self.path_score = result if self.valid else math.inf
