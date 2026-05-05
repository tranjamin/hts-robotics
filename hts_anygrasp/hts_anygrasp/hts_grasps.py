# regular imports
from __future__ import annotations
import numpy as np
import math
import time
import matplotlib.cm as cm
from scipy.spatial.transform import Rotation

try: # ros imports
    from geometry_msgs.msg import Pose
    import open3d as o3d

    # hts robotics imports
    from hts_msgs.action import ComputeGraspValidity, RequestGrasp
    from graspnetAPI import GraspGroup
    from graspnetAPI.grasp import Grasp as GraspNetGrasp
except ImportError as e:
    print(f"Received Import Error {e}, continuing")


class HTSGrasp():
    def __init__(self, grasp: GraspNetGrasp, id):
        # the grasp net objects
        self.grasp_object: GraspNetGrasp = grasp
        self.grasp_score: float = grasp.score

        # the pose object
        self.pose: Pose = None
        self.z: float = 0
        self.theta: float = 0

        self.id: int = id

        # properties of the evaluated grasp
        self.path_score: float = 0.0
        self._is_valid = False
        self._evaluated = False

        self._t0: float = 0.0 # start time
        self.time: float = 0.0 # evaluation time

        # results
        self.result_message = ""
        self.result_err_code = 0
        self.result_err_source = ""
        self.result_err_message = ""
        self.result_timings = []
    
    def set_pose(self, pose: Pose):
        _, _, yaw = Rotation.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w            
        ]).as_euler('xyz', degrees=False)
        
        self.theta = yaw
        self.z = pose.position.z

        self.pose = pose

    def get_pose(self) -> Pose:
        """This grasp's pose"""
        return self.pose
    
    def get_grasp_object(self) -> GraspNetGrasp:
        """This grasp's graspnetapi grasp"""
        return self.grasp_object

    def single_grasp_group(self) -> GraspGroup:
        """A (graspnetapi) grasp group with just this grasp"""
        temp_grasp_group = GraspGroup()
        temp_grasp_group.add(self.get_grasp_object())
        return temp_grasp_group
        
    def validity_request_goal(self, request: ComputeGraspValidity.Request, planning_time: float) -> ComputeGraspValidity.Goal:
        # formats the request
        goal = ComputeGraspValidity.Goal()
        goal.grasp_pose = self.get_pose()
        goal.goal_x = request.goal_x
        goal.goal_y = request.goal_y
        goal.planning_time = planning_time
        goal.target_id = request.id

        # our goal z is expressed as an offset
        goal.goal_z = request.goal_z + goal.grasp_pose.position.z

        return goal

    def process_result(self, result: ComputeGraspValidity.Result):
        """
        Handles the result of an evaluation.
        Sets result fields, validity, evaluation and path score.
        Invalid paths are given infinite path scores
        """
        self.result_message = result.message
        self.result_err_code = result.err_code
        self.result_err_source = result.err_source
        self.result_err_message = result.err_message
        self.result_timings = [result.pickup_ik_time, result.pickup_plan_time, result.pickup_refine_time, result.move_ik_time, result.move_plan_time, result.move_refine_time]
        self._is_valid = result.is_valid
        self.path_score = result.score if result.is_valid else math.inf
        self._evaluated = True
    
    def is_valid(self) -> bool:
        """Is this grasp valid?"""
        return self._is_valid
    
    def pickup_failed(self) -> bool:
        """Did this grasp fail to pickup?"""
        return self.result_err_source == "pickup"

    def move_failed(self) -> bool:
        """Did this grasp fail to move?"""
        return self.result_err_source == "move"
    
    def evaluated(self) -> bool:
        """Has this grasp been evaluated?"""
        return self._evaluated

    def save_grasp_message(self, folder: str, is_flipped: bool=False):
        """Adds a line to grasp_messages.txt with result codes and messages"""
        filename = f"{folder}/grasp_messages.txt" if not is_flipped else f"{folder}/grasp_messages_flipped.txt"
        with open(filename, "a") as f:
            f.write(f"Message {self.result_message} | Err Code {self.result_err_code} | Err Source {self.result_err_source} | Err Msg {self.result_err_message}\r\n")
    
    def start_timer(self):
        """Starts evaluation timer"""
        self._t0 = time.time()
    
    def end_timer(self):
        """Ends evaluation timer"""
        self.time = time.time() - self._t0

    def save_grasp_validity(self, folder: str, include_header: bool=False, is_flipped: bool=False):
        """Saves grasp metrics to /grasp_evaluations.csv"""
        filename = f"{folder}/grasp_evaluations.csv" if not is_flipped else f"{folder}/grasp_evaluations_flipped.csv"
        with open(filename, "a") as f:
            if include_header:
                f.write(f"id,z,th,planning_time,planing_score,grasp_score,pickup_ik_t,pickup_plan_t,pickup_refine_t,move_ik_t,move_plan_t,move_refine_t\r\n") 
            f.write(f"{self.id},{self.z},{self.theta},{self.time},{self.path_score},{self.grasp_score},{str(self.result_timings)[1:-1]}\r\n")
    
    def save_grasp_info(self, folder: str):
        """Saves the grasp positions and orientations to /grasps.txt"""
        pose = self.get_pose()
        with open(f"{folder}/grasps.txt", "a") as f:
            f.write(f"{self.id} | {pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.x} {pose.orientation.y} {pose.orientation.z} {pose.orientation.w}\r\n")

class HTSGraspGroup():
    def __init__(self, grasp_list=None):
        self._grasps: list[HTSGrasp] = [] if grasp_list is None else grasp_list
    
    def get_grasps(self) -> list[HTSGrasp]:
        return self._grasps

    def append(self, hts_grasp: HTSGrasp):
        self._grasps.append(hts_grasp)
        
    def concat(self, hts_grasp_group: HTSGraspGroup):
        self._grasps += hts_grasp_group.get_grasps()
        
    def __len__(self):
        return len(self._grasps)
    
    def num_valid(self) -> int:
        return len([grasp for grasp in self._grasps if grasp.is_valid()])

    def save_metrics(self, folder: str, is_flipped: bool=False):
        """Aggregates the results of the grasps and saves it to /grasp_metrics.txt"""

        valid_grasps = [grasp for grasp in self._grasps if grasp.is_valid()]
        valid_scores = [grasp.path_score for grasp in valid_grasps]
        total_time = sum([grasp.time for grasp in self._grasps])

        filename = f"{folder}/grasp_metrics.txt" if not is_flipped else f"{folder}/grasp_metrics_flipped.txt"
        with open(filename, "a") as f:
            f.write(f"Total Grasps: {len(self)}\r\n")
            f.write(f"Valid Grasps: {len(valid_grasps)}\r\n")
            f.write(f"Percent Valid Grasps: {(len(valid_grasps) / len(self)) if len(self) else '-'}\r\n")
            f.write(f"Total Planning Time: {total_time}\r\n")
            f.write(f"Average Planning Time: {(total_time / len(self)) if len(self) else '-'}\r\n")
            if len(valid_grasps):
                f.write(f"Average Planning Distance: {sum(valid_scores) / len(valid_grasps) if len(valid_grasps) else '-'}\r\n")
            f.write(f"Shortest Planning Distance: {min(valid_scores) if len(valid_grasps) else '-'}\r\n")

    def best_grasp(self) -> HTSGrasp:
        self._grasps.sort(key=lambda x: x.path_score)
        return self._grasps[0]
    
    def valid_grasp_group(self) -> HTSGraspGroup:
        return HTSGraspGroup([grasp for grasp in self._grasps if grasp.is_valid()])
    
    def filter_grasp_group(self, filter) -> HTSGraspGroup:
        return HTSGraspGroup([grasp for grasp in self._grasps if filter(grasp)])

    def visualise(self, cloud, origin_position=[0,0,0], description="HTS Grasp Group"):
        valid_grasps = [grasp for grasp in self._grasps if grasp.is_valid()]

        min_score = min([grasp.path_score for grasp in valid_grasps]) if len(valid_grasps) else 0
        max_score = max([grasp.path_score for grasp in valid_grasps]) if len(valid_grasps) else 0
        
        gg = GraspGroup()

        for hts_grasp in self._grasps:
            gg.add(hts_grasp.get_grasp_object())
        
        grippers = gg.to_open3d_geometry_list()

        trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        for ind, g in enumerate(grippers):
            g.transform(trans_mat)
            if not self._grasps[ind].is_valid():
                color = np.array([[0.0], [0.0], [0.0]], dtype=np.float64)
                g.paint_uniform_color(color)
            elif min_score != max_score:
                color = np.array(cm.RdYlGn_r((self._grasps[ind].path_score - min_score)/(max_score - min_score))[:3], dtype=np.float64)
                g.paint_uniform_color(color)

        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1,      # length of the axes
            origin=origin_position
        )

        cloud.transform(trans_mat)

        o3d.visualization.draw_geometries([*grippers, cloud, origin_frame], window_name=description)
