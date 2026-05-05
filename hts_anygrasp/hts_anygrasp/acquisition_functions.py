from abc import ABC, abstractmethod
import numpy as np
import random

class AcquisitionFunction(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def sample(self, **kwargs) -> int:
        pass

    @abstractmethod
    def time_step(self) -> None:
        pass

class SequentialAcquisition(AcquisitionFunction):
    def __init__(self):
        super().__init__()
        self.current_idx = 0
    
    def sample(self, **kwargs):
        return self.current_idx
    
    def time_step(self) -> None:
        self.current_idx += 1
    

class EpsilonGreedyUCB(AcquisitionFunction):
    def __init__(self, kappa: float=3, eps: float=0.1, eps_final: float | None=None, eps_decay_rate: float=0.99):
        super().__init__()

        assert eps_decay_rate > 0 and eps_decay_rate < 1
        assert eps >= 0 and eps <= 1
        if eps_final is not None:
            assert (eps_final >= 0 and eps_final <= 1) 

        self.kappa = kappa

        # a bigger epsilon tends to more exploration
        self.current_eps: float = eps
        self.eps_start: float = eps
        self.eps_final: float | None = eps_final
        self.eps_decay: float = eps_decay_rate

    def sample(self, mean: np.ndarray | None=None, var: np.ndarray | None=None, uncertainty_weightings: np.ndarray | None=None, logger=None, **kwargs) -> int:
        assert mean is not None and var is not None and uncertainty_weightings is not None
        
        weights = mean + self.kappa*np.sqrt(var) # calculate UCB
        weights = weights - np.min(weights) # make nonnegative
        weights = weights * uncertainty_weightings # weight by uncertainties

        if logger:
            logger.info(f"Weight Sum: {np.sum(weights)}")

        if random.random() > self.current_eps or np.sum(weights) == 0.0:
            return int(np.argmax(weights))
        else:
            return random.choices(range(weights.shape[0]), weights=list(weights))[0]
    
    def time_step(self):
        if self.eps_final is not None:
            if self.eps_final > self.eps_start: # exponential increase towards exploration
                self.current_eps = max(self.eps_final, self.current_eps / self.eps_decay)
            elif self.eps_start > self.eps_final: # exponential increase towards exploitation
                self.current_eps = max(self.eps_final, self.current_eps * self.eps_decay)
