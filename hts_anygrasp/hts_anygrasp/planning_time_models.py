from abc import ABC, abstractmethod
import numpy as np
import scipy

class PlanningTimeModel(ABC):
    def __init__(self, planning_multiplier=2.0, **kwargs):
        self.planning_data: np.ndarray = np.array([])
        self.planning_multiplier: float = planning_multiplier

    @abstractmethod
    def add_planning_data(self, t: float):
        pass

    @abstractmethod
    def validity_failure_model(self, t: float) -> float:
        pass

    @abstractmethod
    def get_initial_planning_time(self) -> float:
        pass

    def get_planning_multiplier(self) -> float:
        return self.planning_multiplier

class LognormalPlanningTimeModel(PlanningTimeModel):
    def __init__(self, planning_multiplier: float=2.0, initial_planning_time: float=0.03, initial_var: float=0.5):
        super().__init__(planning_multiplier)
        self._initial_planning_time: float = initial_planning_time
        self.planning_time_mean: float = np.log(initial_planning_time)
        self.planning_time_variance: float = initial_var

    def get_initial_planning_time(self) -> float:
        return self._initial_planning_time

    def add_planning_data(self, t: float):
        self.planning_data = np.append(self.planning_data, t)
        self.planning_time_mean = 1/self.planning_data.shape[0]*np.sum(np.log(self.planning_data))
        if self.planning_data.shape[0] > 2:
            self.planning_time_variance = 1/self.planning_data.shape[0]*np.sum(np.square(np.log(self.planning_data) - self.planning_time_mean))

    def validity_failure_model(self, t: float):
        return LognormalPlanningTimeModel._compute_norm(t, self.planning_time_mean, self.planning_time_variance)

    @staticmethod
    def _compute_norm(t: float, mean, var):
        return float(scipy.stats.lognorm.cdf(t, np.sqrt(var), scale=np.exp(mean)))