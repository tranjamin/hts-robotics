from __future__ import annotations
import math
import matplotlib.pyplot as plt
import time
import random
import numpy as np
import scipy
import random as random
import functools
from sklearn.gaussian_process.kernels import Matern, RBF
import matplotlib.cm as cm

file = "grasps_planning.npz"
data = np.genfromtxt(file, delimiter=",", skip_header=1)
GRASPS_Z = data[:, 0]
GRASPS_TH = data[:, 1]
GRASPS_PATH_SCORE = data[:, 3]
GRASPS_GRASP_SCORE = data[:, 4]

# GRASPS_Z = np.load("z.npy")
# GRASPS_TH = np.load("th.npy")

from scipy.spatial.distance import cdist, pdist, squareform

def wrapped_sqeuclidean(x, y):
    # return (x[0] - y[0])**2 + ((x[1] - y[1] + np.pi) % (2*np.pi) - np.pi)**2
    return (x[0] - y[0])**2 + (x[1] - y[1])**2

class WrappedRBF(RBF):
    def __init__(self, length_scale=1.0, length_scale_bounds=(1e-5, 1e5)):
        super().__init__(length_scale=length_scale, length_scale_bounds=length_scale_bounds)

    def __call__(self, X, Y=None, eval_gradient=False):
        X = np.atleast_2d(X)
        length_scale = self.length_scale
        if Y is None:
            dists = pdist(X / length_scale, metric=wrapped_sqeuclidean)
            K = np.exp(-0.5 * dists)
            # convert from upper-triangular matrix to square matrix
            K = squareform(K)
            np.fill_diagonal(K, 1)
        else:
            dists = cdist(X / length_scale, Y / length_scale, metric=wrapped_sqeuclidean)
            K = np.exp(-0.5 * dists)

        if eval_gradient:
            if self.hyperparameter_length_scale.fixed:
                # Hyperparameter l kept fixed
                return K, np.empty((X.shape[0], X.shape[0], 0))
            elif not self.anisotropic or length_scale.shape[0] == 1:
                K_gradient = (K * squareform(dists))[:, :, np.newaxis]
                return K, K_gradient
            elif self.anisotropic:
                # We need to recompute the pairwise dimension-wise distances
                K_gradient = (X[:, np.newaxis, :] - X[np.newaxis, :, :]) ** 2 / (
                    length_scale**2
                )
                K_gradient *= K[..., np.newaxis]
                return K, K_gradient
        else:
            return K

# construct a problem space
class ProblemSpace():
    lambda1: float = 1.0
    lambda2: float = 30.0
    
    weibull_k: float = 0.7
    weibull_gamma: float = 0.5
    weibull_base: float = 1.3

    select_greedy_eps: float = 0.7
    
    total_max_time = 100.0
    
    kernel = Matern(length_scale=[0.01, 0.4], nu=3.5)
    # kernel = 
    
    def __init__(self):        
        self.points: list[ProblemPoints] = [ProblemPoints(GRASPS_Z[i], GRASPS_TH[i]) for i in range(len(GRASPS_TH))] # points

        for i in range(len(self.points)):
            self.points[i].path_ret = GRASPS_PATH_SCORE[i]
            self.points[i].grasp_score = 1

        self.t0 = 0.0
        self.N = len(self.points)
        self.k = 1.0

        self.max_path_score = 10

        self.all_z = np.array([p.z for p in self.points]).reshape((len(self.points), 1))
        self.all_th = np.array([p.theta for p in self.points]).reshape((len(self.points), 1))
        self.all_coords = np.hstack((self.all_z, self.all_th))

        self.all_x = self.all_z*np.cos(self.all_th)
        self.all_y = self.all_z*np.sin(self.all_th)

        self.x_mesh, self.y_mesh = np.meshgrid(
            np.linspace(np.min(self.all_x), np.max(self.all_x), 80), 
            np.linspace(np.min(self.all_y), np.max(self.all_y), 80)
        )
        self.x_mesh = np.array(self.x_mesh).reshape((-1, 1))
        self.y_mesh = np.array(self.y_mesh).reshape((-1, 1))
        self.z_mesh = np.sqrt(self.x_mesh**2 + self.y_mesh**2)
        self.th_mesh = np.atan2(self.y_mesh, self.x_mesh)

        mesh_filter = (self.z_mesh >= np.min(self.all_z)) & (self.z_mesh <= np.max(self.all_z))

        self.x_mesh = self.x_mesh[mesh_filter].reshape((-1, 1))
        self.y_mesh = self.y_mesh[mesh_filter].reshape((-1, 1))
        self.z_mesh = self.z_mesh[mesh_filter].reshape((-1, 1))
        self.th_mesh = self.th_mesh[mesh_filter].reshape((-1, 1))

        self.est_path_scores = np.zeros((self.N, 1))
        self.cov = np.eye(self.N)
    
    def start_timer(self):
        self.t0 = time.time()
    
    def select_next(self):
        # construct training dataset from already sampled points
        weights = (self.est_path_scores.ravel() + 3*np.sqrt(np.diag(self.cov)))*np.array([1 - p.certainty for p in self.points])
        weights = weights - np.min(weights)

        # weights = (self.est_path_scores.ravel() + 3*np.sqrt(np.diag(self.cov)))
        # weights = weights - np.min(weights)

        if np.sum(weights) == 0.0:
            idx = random.choices(range(len(self.points)))[0]
            return idx, self.points[idx]

        # sample
        idx = random.choices(range(len(self.points)), weights=list(weights))[0]
        return idx, self.points[idx]
        
    def plot_grasps(self):
        mesh_mean, mesh_cov = self.get_preds(np.hstack((self.z_mesh, self.th_mesh)))
        mesh_uncertanties = np.sqrt(np.diag(mesh_cov))
        # mesh_mean = self.max_path_score - mesh_mean

        costs_estimated = self.est_path_scores        
        uncertanties = np.sqrt(np.diag(self.cov))

        # evaluated = [x.evaluated for x in self.points]

        true_costs = [x.cost(self.max_path_score) for x in self.points]
        true_path_scores = [x.path_ret if not math.isinf(x.path_ret) else 0.0 for x in self.points]
        calculated_path_scores = [0.0 if not x.evaluated else x.path_score for x in self.points]
        errs = [costs_estimated[i] - true_costs[i] for i in range(len(true_costs))]

        weights = (self.est_path_scores.ravel() - self.k*np.sqrt(np.diag(self.cov)))

        fig, axs = plt.subplots(2, 3, figsize=(24, 12), subplot_kw={'projection': 'polar'})
        splt1 = axs[0, 0].scatter(
            self.all_th, self.all_z, vmin=0.0, vmax=1.0,
            c=uncertanties, cmap=cm.RdYlGn_r,marker="o", linewidths=1, edgecolors="black",
            )
        splt2 = axs[0, 1].scatter(
            self.all_th, self.all_z, vmin=0.0, vmax=1.0,
            c=costs_estimated, cmap=cm.RdYlGn_r,marker="o", linewidths=1, edgecolors="black")
        splt3 = axs[0, 2].scatter(self.all_th, self.all_z, c=true_path_scores, cmap=cm.RdYlGn_r,marker="o", linewidths=1, edgecolors="black")

        splt4 = axs[1, 0].scatter(self.th_mesh, self.z_mesh, c=mesh_uncertanties, cmap=cm.RdYlGn_r,marker=",")
        splt1 = axs[1, 0].scatter(self.all_th, self.all_z, c=uncertanties, cmap=cm.RdYlGn_r,marker="o", linewidths=1, edgecolors="black")
        splt5 = axs[1, 1].scatter(self.th_mesh, self.z_mesh, c=mesh_mean, cmap=cm.RdYlGn_r,marker=",")
        splt6 = axs[1, 2].scatter(self.all_th, self.all_z, c=errs, cmap=cm.RdYlGn_r,marker="o", linewidths=1, edgecolors="black")

        axs[0, 0].set_title("Variance")
        axs[0, 1].set_title("Mean Cost Score")
        axs[0, 2].set_title("True Path Scores")

        axs[1, 0].set_title("Variance Map")
        axs[1, 1].set_title("Mean Cost Map")
        axs[1, 2].set_title("Errs")

        fig.colorbar(splt1, ax=axs[0,0])
        fig.colorbar(splt2, ax=axs[0,1])
        fig.colorbar(splt3, ax=axs[0,2])
        fig.colorbar(splt4, ax=axs[1,0])
        fig.colorbar(splt5, ax=axs[1,1])
        fig.colorbar(splt6, ax=axs[1,2])

        # plt.savefig(f"{time.time()}_plt.png")
        plt.show()
        
    def choose_best(self) -> ProblemPoints:
        print("Choose Best")
        # cost = [x.cost(self.max_path_score) for x in self.points]
        # max_cost = max(cost)
        # cost = [(1 if x == max_cost else 0) for x in cost]
        # return random.choices(self.points, weights=cost)[0]
        
    def update_map(self):
        # construct training dataset from already sampled points
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))
        evaluated_noise = np.array([1 - p.certainty for p in evaluated_points])

        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(self.all_coords, evaluated_coords)
        Kxx = self.kernel(self.all_coords)

        # Sigma = np.eye(len(evaluated_points))
        Sigma = np.diag([(1 - p.known_certainty)*np.max(self.cov) for p in evaluated_points])
        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        
        self.est_path_scores = KxX @ np.linalg.inv(KXX + Sigma) @ y
        self.cov = Kxx - KxX @ np.linalg.inv(KXX + Sigma) @ KxX.T

        np.clip(self.cov, 0.0, None, self.cov)
    
    def get_preds(self, coords):
        evaluated_points = list(filter(lambda p: p.evaluated, self.points))
        n = len(evaluated_points)
        evaluated_z = np.array([p.z for p in evaluated_points]).reshape((n, 1))
        evaluated_th = np.array([p.theta for p in evaluated_points]).reshape((n, 1))
        evaluated_coords = np.hstack((evaluated_z, evaluated_th))
        evaluated_noise = np.array([1 - p.certainty for p in evaluated_points])

        KXX = self.kernel(evaluated_coords)
        KxX = self.kernel(coords, evaluated_coords)
        Kxx = self.kernel(coords)

        y = np.array([p.norm_path_score(self.max_path_score) for p in evaluated_points]).reshape((n, 1))
        u = KxX @ np.linalg.inv(KXX) @ y
        c = Kxx - KxX @ np.linalg.inv(KXX) @ KxX.T
        return u,c
        
    def update_max_path_score(self, newest_score):
        # never update on an invalid score
        if math.isinf(newest_score) or newest_score == 0:
            return

        # if we haven't found a valid path yet:
        if self.max_path_score == 0:
            self.max_path_score = newest_score
        elif newest_score > self.max_path_score:
            self.max_path_score = newest_score

    @staticmethod
    def validity_failure_model(t: float):
        # calculates the probability that the failure was real
        return float(scipy.stats.weibull_min.cdf(ProblemSpace.weibull_base**t - 1, ProblemSpace.weibull_k, scale=ProblemSpace.weibull_gamma))
        return 1.0
            
    def send_goal(self):
        # print("Sending Goal")
        idx, point = self.select_next()
        
        if point.evaluated:
            print("Re-evaluating a grasp")

        # print(f"Candidate Grasp {idx + 1}/{len(self.points)}")
        # print(f"Grasp has position {point.z} {point.theta}")

        point = self.points[idx]
        point.handle_evaluation_result()
        self.update_max_path_score(point.path_score)
        print(f"------- Evaluated idx {idx} ----------")
        norm_path_score = point.norm_path_score(self.max_path_score)
        print(f"Certainty is now {point.certainty}, Path Score is now {norm_path_score}")


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

        self.path_ret = 0

        self.is_reflected: bool # whether we are dealing in the reflected space or not
        self.grasp_score: float = 1.0 # the score output from anygrasp
        
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
        # if math.isinf(self.path_score):
        #     return -max_path_score*self.grasp_score
        # else:
        return self.norm_path_score(max_path_score)*self.grasp_score

    def norm_path_score(self, max_path_score):
        # if we are invalid, we say that the cost is twice is longest path cost
        # if math.isinf(self.path_score):
        #     return -max_path_score*self.grasp_score
        # else:
        return max(max_path_score - self.path_score, -max_path_score)

    def handle_evaluation_result(self):
        self.evaluated = True
        # self.valid = bool(math.cos(self.theta)*self.z > 0)
        self.valid = not math.isinf(self.path_ret)
        self.update_certainty_upon_eval()
        self.path_score = self.path_ret if self.valid else math.inf

if __name__ == "__main__":
    ps = ProblemSpace()
    for i in range(400):
        ps.send_goal()

        if i % 10 == 0:
            ps.plot_grasps()
