from abc import ABC, abstractmethod
import numpy as np
import numpy.typing as npt
import scipy
import functools

class PlanningTimeModel(ABC):
    """
    An abstract class which models the planning time distribution
    """
    def __init__(self, planning_multiplier: float=2.0, **kwargs):
        """
        Parameters:
            planning_multiplier: the rate at which the allocated planning time increases upon evaluation failure
        """
        self.planning_data: npt.NDArray[np.float64] = np.array([])
        self.planning_multiplier: float = planning_multiplier
        
    @abstractmethod
    def add_planning_data(self, t: float) -> None:
        """Adds another data point to the planning time model"""
        pass

    @abstractmethod
    def register_failed_plan(self) -> None:
        """Registers that plan failed to plan"""

    @abstractmethod
    def validity_failure_model(self, t: float) -> float:
        """Calculates the likelihood that a grasp would have failed before time t"""
        pass

    @abstractmethod
    def get_exploration_planning_time(self) -> float:
        """Gets the exploration planning time"""
        pass

    @abstractmethod
    def get_initial_planning_time(self) -> float:
        """Gets the default planning time"""
        pass

    def get_planning_multiplier(self) -> float:
        """Gets the planning multiplier"""
        return self.planning_multiplier

class LognormalPlanningTimeModel(PlanningTimeModel):
    """A model which assumes that planning time is lognormally distributed"""

    def __init__(self, planning_multiplier: float=2.0, initial_planning_time: float=0.03, initial_var: float=0.5):
        """
        Parameters:
            planning_multiplier: the rate at which the allocated planning time increases upon evaluation failure
            initial_planning_time: the base planning time allocated for each evaluation
            initial_var: the inital assumed variance of the distribution
        """
        super().__init__(planning_multiplier=planning_multiplier)
        self._initial_planning_time: float = initial_planning_time
        self.planning_time_mean: float = np.log(initial_planning_time)
        self.planning_time_variance: float = initial_var
        self.current_exploration_planning_time: float = initial_planning_time

    def register_failed_plan(self) -> None:
        self.current_exploration_planning_time *= self.planning_multiplier

    def get_exploration_planning_time(self) -> float:
        return self.current_exploration_planning_time

    def get_initial_planning_time(self) -> float:
        """Gets the base planning time allocated for each evaluation"""
        return self._initial_planning_time

    def add_planning_data(self, t: float) -> None:
        """Adds another data point to the planning time model"""
        self.planning_data = np.append(self.planning_data, t)

        # recalculate estimates of mean and variance
        self.planning_time_mean = 1/self.planning_data.shape[0]*np.sum(np.log(self.planning_data))
        if self.planning_data.shape[0] > 2:
            self.planning_time_variance = 1/self.planning_data.shape[0]*np.sum(np.square(np.log(self.planning_data) - self.planning_time_mean))

    def validity_failure_model(self, t: float) -> float:
        """Calculates the certainty that a failure at time t is a true failure"""
        return LognormalPlanningTimeModel._compute_cdf(t, self.planning_time_mean, self.planning_time_variance)

    @staticmethod
    @functools.cache
    def _compute_cdf(t: float, mean: float, var: float) -> float:
        """
        Computes the CDF of the lognormal
        
        Parameters:
            t: the cutoff for the CDF
            mean: the mean of the distribution
            var: the variance of the distribution
        """
        return float(scipy.stats.lognorm.cdf(t, np.sqrt(var), scale=np.exp(mean)))