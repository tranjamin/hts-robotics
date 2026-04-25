from __future__ import annotations
import math
import matplotlib.pyplot as plt
import time
import random
import numpy as np
import scipy
import functools

GRASPS_Z = np.load("z.npy")
GRASPS_TH = np.load("th.npy")

# construct a problem space
class ProblemSpace():
    lambda1: float = 1.0
    lambda2: float = 30.0
    
    weibull_k: float = 0.7
    weibull_gamma: float = 0.5
    weibull_base: float = 1.3

    sigma_theta: float = 0.4
    sigma_z: float = 0.05
    
    select_greedy_eps: float = 0.7

    certainty_scaler_range: float = 1e-3
    prediction_z_range: float = 0.05
    prediction_th_range: float = math.pi/4
    
    total_max_time = 100.0
    
    def __init__(self):        
        self.points: list[ProblemPoints] = [ProblemPoints(GRASPS_Z[i], GRASPS_TH[i]) for i in range(len(GRASPS_TH))] # points
        self.t0 = 0.0

        self.max_path_score = 0.0
    
    def start_timer(self):
        self.t0 = time.time()
    
    def select_next(self):
        # print("Select Next")
        # highest uncertainty
        uncertanties = [1 - x.get_certainty() for x in self.points]
        costs = [x.cost(self.max_path_score) for x in self.points]
        if min(costs) < 0:
            costs = [x - min(costs) for x in costs]
        weighted_cost = [costs[i]*uncertanties[i] for i in range(len(costs))]

        zs = np.array([p.z for p in self.points])
        thetas = np.array([p.theta for p in self.points])

        x = zs*np.cos(thetas)
        y = zs*np.sin(thetas)

        # fig, axs = plt.subplots(1, 2)

        # splt1 = axs[0].scatter(x, y, c=weighted_cost)
        # splt2 = axs[1].scatter(x, y, c=uncertanties)

        # axs[0].set_title("Weighted Cost")
        # axs[1].set_title("Uncertainties")
        
        # fig.colorbar(splt1, ax=axs[0])
        # fig.colorbar(splt2, ax=axs[1])

        # plt.show()


        # if everything is uncertain, then select greedily
        if sum(weighted_cost) != 0.0 and (min(uncertanties) == 1.0 or random.random() < self.select_greedy_eps):
            if random.random() < 0.5:
                # print(f"Trying to sample greedily")
                idx = random.choices(range(len(self.points)), weights=weighted_cost)[0]
                print(f"Selecting probabilistic greedily {idx}")
                return idx, self.points[idx]
            else:
                # print(f"Trying to sample greedily")
                idx = weighted_cost.index(max(weighted_cost))
                print(f"Selecting greedily {idx}")
                return idx, self.points[idx]
        elif sum(uncertanties) != 0.0:
            if random.random() < 0.5:
                # print(f"Trying to sample uncertainty")
                idx = random.choices(range(len(self.points)), weights=uncertanties)[0]
                print(f"Selecting greedily based on uncertainty {idx}")
                return idx, self.points[idx]
            else:
                # print(f"Trying to sample greedily")
                idx = uncertanties.index(max(uncertanties))
                print(f"Selecting based on uncertainty {idx}")
                return idx, self.points[idx]
        else:
            print("Everything is 100%% certain now")
            pass # do something
        
    def plot_grasps(self):
        zs = np.array([p.z for p in self.points])
        thetas = np.array([p.theta for p in self.points])

        x = zs*np.cos(thetas)
        y = zs*np.sin(thetas)

        path_scores = [min(self.max_path_score, x.path_score) for x in self.points]
        grasp_scores = [x.grasp_score for x in self.points]
        
        uncertanties = [1 - x.get_certainty() for x in self.points]
        costs = [x.cost(self.max_path_score) for x in self.points]
        if min(costs) < 0:
            costs = [x - min(costs) for x in costs]
        weighted_cost = [costs[i]*uncertanties[i] for i in range(len(costs))]

        evaluated = [x.evaluated for x in self.points]
        distance = [ProblemPoints._distance_scaler(p.z, p.theta) for p in self.points]

        fig, axs = plt.subplots(2, 3, figsize=(24, 12))
        splt1 = axs[0, 0].scatter(x, y, c=uncertanties, vmin=0.0, vmax=1.0)
        splt2 = axs[0, 1].scatter(x, y, c=costs, vmin=0.0)
        splt3 = axs[0, 2].scatter(x, y, c=path_scores, vmin=0.0)
        splt4 = axs[1, 0].scatter(x, y, c=grasp_scores, vmin=0.0)
        splt5 = axs[1, 1].scatter(x, y, c=weighted_cost)
        splt6 = axs[1, 2].scatter(x, y, c=evaluated)

        axs[0, 0].set_title("Uncertainties")
        axs[0, 1].set_title("Costs")
        axs[0, 2].set_title("Path Scores")
        axs[1, 0].set_title("Grasp Scores")
        axs[1, 1].set_title("Weighted Cost")
        axs[1, 2].set_title("Evaluated")

        fig.colorbar(splt1, ax=axs[0,0])
        fig.colorbar(splt2, ax=axs[0,1])
        fig.colorbar(splt3, ax=axs[0,2])
        fig.colorbar(splt4, ax=axs[1,0])
        fig.colorbar(splt5, ax=axs[1,1])
        fig.colorbar(splt6, ax=axs[1,2])

        # plt.savefig(f"{time.time()}_plt.png")
        plt.show()
        
    def choose_best(self) -> ProblemPoints:
        # print("Choose Best")
        cost = [x.cost(self.max_path_score) for x in self.points]
        max_cost = max(cost)
        cost = [(1 if x == max_cost else 0) for x in cost]
        return random.choices(self.points, weights=cost)[0]
        
    def update_map(self):
        # print("Update Map")
        for idx, p in enumerate(self.points):
            orig_certainty = p.certainty
            orig_prediction = p.path_score
            p.update_certainty_by_kde(self.points)
            p.update_prediction_by_kde(self.points, self.max_path_score)
            # print(f"idx {idx}: Updated uncertainty from {orig_certainty} to {p.certainty}")
            # print(f"idx {idx}: Updated path score from {orig_prediction} to {p.path_score}")
        
    def update_max_path_score(self, newest_score):
        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = 2*newest_score
        elif 2*newest_score > self.max_path_score:
            self.max_path_score = 2*newest_score

    @staticmethod
    def validity_failure_model(t: float):
        # calculates the probability that the failure was real
        return float(scipy.stats.weibull_min.cdf(ProblemSpace.weibull_base**t - 1, ProblemSpace.weibull_k, scale=ProblemSpace.weibull_gamma))
        # return 1.0
            
    def send_goal(self):
        # print("Sending Goal")
        idx, point = self.select_next()
        
        if point.evaluated:
            print("Re-evaluating a grasp")

        # print(f"Candidate Grasp {idx + 1}/{len(self.points)}")
        # print(f"Grasp has position {point.z} {point.theta}")

        point = self.points[idx]
        point.handle_evaluation_result()
        print(f"------- Evaluated idx {idx} ----------")
        cost = point.cost(math.inf)
        print(f"Certainty is now {point.certainty}, Path Score is now {point.path_score}, Cost {cost}")

        self.update_max_path_score(point.path_score)

        self.update_map()

        # show the updated map
        # self.plot_grasps()
            
class ProblemPoints():
    INITIAL_PLANNING_TIME = 0.3
    
    def __init__(self, z, theta):
        self.known_certainty: float = 0 # the known certainty of the path score
        self.certainty: float = 0 # how certain we are of the path score        
        
        self.theta: float = theta
        self.z: float = z
        
        self.is_reflected: bool # whether we are dealing in the reflected space or not
        self.grasp_score: float = 0 # the score output from anygrasp
        
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
            self.known_certainty = ProblemSpace.validity_failure_model(self.max_planning_time)
            self.certainty = self.known_certainty
            self.max_planning_time *= 2 # double the planning time next time
    
    def cost(self, max_path_score):
        # if we are invalid, we say that the cost is twice is longest path cost
        return -ProblemSpace.lambda1*min(self.path_score, max_path_score) + ProblemSpace.lambda2*self.grasp_score
 
    @functools.cache
    @staticmethod
    def _certainty_scaler_at_offset(z_offset: float, theta_offset: float):
        # we multiply our certainty with a scaled gaussian
        z_scaler = 2*scipy.stats.norm.sf(z_offset, scale=ProblemSpace.sigma_z)
        theta_scaler = 2*scipy.stats.norm.sf(theta_offset, scale=ProblemSpace.sigma_theta)        
        return float(z_scaler * theta_scaler)
    
    @functools.cache
    @staticmethod
    def _distance_scaler(z_offset: float, theta_offset: float):
        z_scale = max(1 - abs(z_offset)/ProblemSpace.prediction_z_range, 0)
        th_scale = max(1 - abs(theta_offset)/ProblemSpace.prediction_th_range, 0)
        # return math.sqrt(z_scale**2 + th_scale**2)
        if z_scale*th_scale == 0.0:
            return 0.0
        
        return float(z_scale*th_scale/2 + 0.5)
    
    def update_certainty_by_kde(self, points: list[ProblemPoints]):
        if self.evaluated and self.valid:
            # if we have already evaluated this positively, then do not update
            return
        
        # if we have already evaluated this negatively, then we update normally
        if self.evaluated and not self.valid:
            pass

        # if we know nothing about this point, then we use KDE
        # of if we have tried and failed
        positive_contributions = 0.0
        positive_weights = 0.0
        negative_contributions = 0.0
        negative_weights = 0.0
        for i, p in enumerate(points):
            # if p == self:
                # we seed it with the current estimate of certainty
                # all_contributions += 1.0 * self.certainty
                # all_weights += 1.0
                # continue
            if not p.evaluated and p != self:
                continue
            dtheta = abs(p.theta - self.theta) % (2*math.pi)
            dtheta = min(dtheta, 2*math.pi - dtheta)
            
            # this is how much an evaluated point is related to our point
            k = ProblemPoints._certainty_scaler_at_offset(abs(p.z - self.z), dtheta)

            # # don't add a contribution if it's too far away
            # if (k < ProblemSpace.certainty_scaler_range):
            #     continue

            # we store the positive and the negative contributions separately
            if p.valid:
                positive_weights += k
                positive_contributions += k*p.known_certainty
            else:
                negative_weights += k
                if p.evaluated:
                    negative_contributions += k*p.known_certainty
                else:
                    negative_contributions += k*p.certainty

        total_weights = (positive_weights + negative_weights)
        total_contributions = abs(positive_contributions - negative_contributions)
            
        self.certainty = total_contributions/total_weights if total_weights != 0.0 else 0.0
    
    def update_prediction_by_kde(self, points: list[ProblemPoints], path_max):
        if self.evaluated and self.valid:
            # if we have already evaluated this positively, then do not update
            return
        
        # if we have already evaluated this negatively, then we update normally
        if self.evaluated and not self.valid:
            pass

        # we start by seeding it with the current prediction
        all_contributions = 0.0
        all_weights = 0.0
        for i, p in enumerate(points):
            if not p.evaluated and p != self:
                continue
            k = ProblemPoints._distance_scaler(p.z - self.z, p.theta - self.theta)
            
            # scale all weights by by the certainty of the point
            if p.evaluated:
                all_weights += k*p.known_certainty
                all_contributions += k*min(p.path_score, path_max)*p.known_certainty

            # print(f"Adding contribution from {i}: distance scaler {k}")
        self.path_score = all_contributions/all_weights if all_weights != 0.0 else path_max*2
        if self.path_score == 0:
            self.path_score = 2*path_max

        # scale in the existing knowledge
        self.path_score = self.path_score*(1 - self.known_certainty) + self.known_certainty*2*path_max
    
    def handle_evaluation_result(self):
        self.evaluated = True
        self.valid = bool(math.cos(self.theta)*self.z > 0)
        self.update_certainty_upon_eval()
        self.path_score = 10.0 if self.valid else math.inf

if __name__ == "__main__":
    ps = ProblemSpace()
    for i in range(100):
        ps.send_goal()
        if not i % 20:
            ps.plot_grasps()