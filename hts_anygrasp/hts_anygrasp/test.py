from __future__ import annotations
import numpy as np
from grasp_selection import GPPointSelector, EpsilonGreedyUCB, LognormalPlanningTimeModel, CoordinatePoint
from utils import ValidityContext

file = "grasps_planning.npz"
data = np.genfromtxt(file, delimiter=",", skip_header=1)
GRASPS_Z = data[:, 1]
GRASPS_TH = data[:, 2]
GRASPS_PATH_SCORE = data[:, 4]
GRASPS_GRASP_SCORE = data[:, 5]

tuner = LognormalPlanningTimeModel()
acquisition_fn = EpsilonGreedyUCB(kappa=3, eps=0.1, eps_final=None)

if __name__ == "__main__":
    points = []
    for i in range(len(GRASPS_Z)):
        point = CoordinatePoint((GRASPS_Z[i], GRASPS_TH[i]))
        point.set_known_path_score(GRASPS_PATH_SCORE[i])
        points.append(point)

    ps = GPPointSelector(points, tuner, acquisition_fn, ValidityContext(None))
    ps._handle_validity_send_goal()