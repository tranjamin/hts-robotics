from abc import ABC, abstractmethod
import numpy as np
import numpy.typing as npt
import random
from typing import Any

class AcquisitionFunction(ABC):
    """
    An abstract class which models an acquisition function for selecting points to evaluate    
    """
    def __init__(self):
        pass

    @abstractmethod
    def sample(self, **kwargs) -> int:
        """Samples an index to evaluate"""
        pass

    @abstractmethod
    def time_step(self) -> None:
        """Steps the acquisition function forward in time"""
        pass

class SequentialAcquisition(AcquisitionFunction):
    """
    An acquisition function which samples points in canonical order
    """
    def __init__(self):
        super().__init__()
        self.current_idx: int = 0 # the next point to sample
    
    def sample(self, **kwargs) -> int:
        return self.current_idx
    
    def time_step(self) -> None:
        self.current_idx += 1
    

class EpsilonGreedyUCB(AcquisitionFunction):
    """
    An acquisition function which samples points according to the upper confidence bound
    """

    def __init__(self, kappa: float=3, eps: float=0.1, eps_final: float | None=None, eps_decay_rate: float=0.99):
        """
        Parameters:
            kappa: the weighting between mean and standard deviation in the UCB
            eps: the sampler chooses the highest UCB with probability (1 - eps)
            eps_final: the final value for epsilon, or None if eps will stay constant
            eps_decay_rate: the rate at which epsilon decays, or the rate that it increases at if eps_final > eps
        """
        super().__init__()

        # assert the bounds of the parameters
        assert eps_decay_rate > 0 and eps_decay_rate < 1
        assert eps >= 0 and eps <= 1
        if eps_final is not None:
            assert (eps_final >= 0 and eps_final <= 1) 

        self.kappa: float = kappa
        self.current_eps: float = eps
        self.eps_start: float = eps
        self.eps_final: float | None = eps_final
        self.eps_decay: float = eps_decay_rate

    def sample(self, 
               mean: npt.NDArray[np.float64] | None=None, 
               var: npt.NDArray[np.float64] | None=None, 
               uncertainty_weightings: npt.NDArray[np.float64] | None=None, 
               **kwargs
               ) -> int:
        """
        Samples an index to evaluate

        Parameters:
            mean: the means of candidate points
            var: the variance of candidate points
            uncertainty_weightings: additional weightings for each sample
        Returns:
            the index of the point to evaluate
        """

        # ensure everything is finite
        try:
            assert mean is not None and var is not None and uncertainty_weightings is not None
            assert np.all(np.isfinite(mean)) and np.all(np.isfinite(var)) and np.all(np.isfinite(uncertainty_weightings))
        except AssertionError:
            raise AssertionError("EpsilonGreedyUCB encountered an infinite value when sampling")
        
        # calculate weightings
        weights = mean + self.kappa*np.sqrt(var) # calculate UCB
        weights = weights - np.min(weights) # make nonnegative
        weights = weights * uncertainty_weightings # weight by uncertainties

        # sample
        if random.random() > self.current_eps or np.sum(weights) == 0.0:
            return int(np.argmax(weights))
        else:
            return random.choices(range(weights.shape[0]), weights=list(weights))[0]
    
    def time_step(self) -> None:
        """
        Steps the epsilon scheduling forward in time
        """
        if self.eps_final is not None:
            if self.eps_final > self.eps_start: # exponential increase towards exploration
                self.current_eps = max(self.eps_final, self.current_eps / self.eps_decay)
            elif self.eps_start > self.eps_final: # exponential increase towards exploitation
                self.current_eps = max(self.eps_final, self.current_eps * self.eps_decay)
