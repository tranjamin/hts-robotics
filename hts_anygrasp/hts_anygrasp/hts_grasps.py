# regular imports
from __future__ import annotations
from typing import Callable, Any
import numpy as np
import numpy.typing as npt
import math
import time
import matplotlib.cm as cm
from scipy.spatial.transform import Rotation

try: # ros imports
    from geometry_msgs.msg import Pose
    import open3d as o3d
    from hts_msgs.action import ComputeGraspValidity, RequestGrasp
    from graspnetAPI import GraspGroup as GraspNetGroup
    from graspnetAPI.grasp import Grasp as GraspNetGrasp
except ImportError as e:
    print(f"Received Import Error {e}, continuing")


class HTSGrasp():
    """A wrapping class for the grasps"""
    def __init__(self, grasp: GraspNetGrasp, id):
        # the grasp net objects
        self.grasp_object: GraspNetGrasp = grasp
        self.grasp_score: float = grasp.score

        self.pose: Pose | None = None # the pose object
        self.z: float = 0 # the vertical displacement of the pose
        self.theta: float = 0 # the yaw of the pose

        self.id: int = id # the id of the pose for logging purposes

        self.path_score: float = 0.0 # the path score
        self._is_valid = False # whether the grasp has been successfully evaluated
        self._evaluated = False # whether the grasp has been evaluated

        self._t0: float = 0.0 # start time
        self.time: float = 0.0 # evaluation time

        # results for logging purposes
        self.result_message: str = ""
        self.result_err_code: int = 0
        self.result_err_source: str = ""
        self.result_err_message: str = ""
        self.result_timings: list[float] = []
    
    def set_pose(self, pose: Pose) -> None:
        """
        Sets the internal pose of the grasp. Also updates z and theta as required.

        Parameters:
            pose: the pose object
        """
        # calculate the yaw of the pose from the quat
        yaw: float
        _, _, yaw = Rotation.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w            
        ]).as_euler('xyz', degrees=False)
        
        # updates z and theta
        self.theta = yaw
        self.z = pose.position.z

        # sets pose
        self.pose = pose

    def get_pose(self) -> Pose:
        """This grasp's pose"""
        return self.pose
    
    def get_grasp_object(self) -> GraspNetGrasp:
        """This grasp's graspnetapi grasp"""
        return self.grasp_object

    def single_grasp_group(self) -> GraspNetGroup:
        """A (graspnetapi) grasp group with just this grasp"""
        temp_grasp_group: GraspNetGroup = GraspNetGroup()
        temp_grasp_group.add(self.get_grasp_object())
        return temp_grasp_group
        
    def evaluation_request_goal(self, request: RequestGrasp.Goal, planning_time: float) -> ComputeGraspValidity.Goal:
        """
        Generates a request goal for evaluating this grasp
        
        Parameters:
            request: the grasp planning request, containing information about the goal pose and target id
            planning_time: the planning time allocated for this evaluation
        Returns:
            a goal to be sent to the evaluation server
        """
        # formats the request
        goal: ComputeGraspValidity.Goal = ComputeGraspValidity.Goal()

        goal.grasp_pose = self.get_pose()
        goal.goal_x = request.goal_x
        goal.goal_y = request.goal_y
        goal.goal_z = request.goal_z + goal.grasp_pose.position.z # our goal z is expressed as an offset
        goal.planning_time = planning_time
        goal.target_id = request.id

        return goal

    def process_result(self, result: ComputeGraspValidity.Result) -> None:
        """
        Handles the result of an evaluation. Sets result fields, validity, evaluation and path score.
        Invalid paths are given infinite path scores

        Parameters:
            result: the result from the grasp validity
        """

        # sets all the results data
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

    def save_grasp_message(self, folder: str, is_flipped: bool=False) -> None:
        """
        Adds a line to /grasp_messages.txt with result codes and messages
        
        Parameters:
            folder: the main directory to save data to
            is_flipped: whether this grasp was flipped or not
        """
        filename: str = f"{folder}/grasp_messages.txt" if not is_flipped else f"{folder}/grasp_messages_flipped.txt"
        with open(filename, "a") as f:
            f.write(f"Message {self.result_message} | Err Code {self.result_err_code} | Err Source {self.result_err_source} | Err Msg {self.result_err_message}\r\n")
    
    def start_timer(self) -> None:
        """Starts evaluation timer"""
        self._t0 = time.time()
    
    def end_timer(self) -> None:
        """Ends evaluation timer. Updates .time"""
        self.time = time.time() - self._t0

    def save_grasp_evaluations(self, folder: str, include_header: bool=False, is_flipped: bool=False) -> None:
        """
        Saves grasp evaluation results to /grasp_evaluations.csv
        
        Parameters:
            folder: the main directory to save data to
            include_header: whether to include column names (i.e., if this was the first row added)
            is_flipped: whether this grasp was flipped or not
        """
        filename: str = f"{folder}/grasp_evaluations.csv" if not is_flipped else f"{folder}/grasp_evaluations_flipped.csv"
        with open(filename, "a") as f:
            if include_header:
                f.write(f"id,z,th,planning_time,planing_score,grasp_score,pickup_ik_t,pickup_plan_t,pickup_refine_t,move_ik_t,move_plan_t,move_refine_t\r\n") 
            f.write(f"{self.id},{self.z},{self.theta},{self.time},{self.path_score},{self.grasp_score},{str(self.result_timings)[1:-1]}\r\n")
    
    def save_grasp_info(self, folder: str) -> None:
        """
        Saves the grasp positions and orientations to /grasps.txt
        
        Parameters:
            folder: the main directory to save data to
        """
        pose = self.get_pose()
        with open(f"{folder}/grasps.txt", "a") as f:
            f.write(f"{self.id} | {self.grasp_score} | {pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.x} {pose.orientation.y} {pose.orientation.z} {pose.orientation.w}\r\n")

class HTSGraspGroup():
    """"A wrapping class for grasp groups"""
    def __init__(self, grasp_list: list[HTSGrasp] | None =None):
        """
        Parameters:
            grasp_list: the list of HTSGrasp objects, or None
        """
        self._grasps: list[HTSGrasp] = [] if grasp_list is None else grasp_list
    
    def get_grasps(self) -> list[HTSGrasp]:
        """Gets the list of HTSGrasp objects"""
        return self._grasps

    def append(self, hts_grasp: HTSGrasp) -> None:
        """Adds a grasp to this group"""
        self._grasps.append(hts_grasp)
        
    def concat(self, hts_grasp_group: HTSGraspGroup) -> None:
        """Concatenates the grasps of another grasp group to this grasp"""
        self._grasps += hts_grasp_group.get_grasps()
        
    def __len__(self):
        return len(self._grasps)
    
    def num_valid(self) -> int:
        """Gets the number of grasps in this group which were evaluated as valid"""
        return len([grasp for grasp in self._grasps if grasp.is_valid()])

    def save_metrics(self, folder: str, is_flipped: bool=False):
        """
        Aggregates the results of the grasps and saves it to /grasp_metrics.txt
        
        Parameters:
            folder: the main directory to save data to
            is_flipped: whether this grasp group contained flipped grasps or not
        """

        valid_grasps: list[HTSGrasp] = [grasp for grasp in self._grasps if grasp.is_valid()]
        valid_scores: list[float] = [grasp.path_score for grasp in valid_grasps]
        total_time: float = sum([grasp.time for grasp in self._grasps])
        filename: str = f"{folder}/grasp_metrics.txt" if not is_flipped else f"{folder}/grasp_metrics_flipped.txt"

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
        """gets the best grasp according to the grasp path scores"""
        self._grasps.sort(key=lambda x: x.path_score)
        return self._grasps[0]
    
    def valid_grasp_group(self) -> HTSGraspGroup:
        """
        Gets all valid grasps
        
        Returns:
            a new HTSGraspGroup containing only valid grasps
        """
        return HTSGraspGroup([grasp for grasp in self._grasps if grasp.is_valid()])
    
    def filter_grasp_group(self, filter: Callable[[HTSGrasp], bool]) -> HTSGraspGroup:
        """
        Filters grasps according to a filter function

        Parameters:
            filter: the function to filter grasps by

        Returns:
            a new HTSGraspGroup containing the filtered grasps
        """
        return HTSGraspGroup([grasp for grasp in self._grasps if filter(grasp)])

    def visualise(self, cloud: o3d.cuda.pybind.geometry.PointCloud, origin_position: list[float]=[0,0,0], description: str="HTS Grasp Group") -> None:
        """
        Displays an o3d visualisation of this grasp group

        Parameters:
            cloud: the point cloud to draw
            origin_position: the position to draw the origin axis from
            description: the title of the visualisation
        """

        # calculates the min and max path score
        valid_grasps: list[HTSGrasp] = [grasp for grasp in self._grasps if grasp.is_valid()]
        min_score: float = min([grasp.path_score for grasp in valid_grasps]) if len(valid_grasps) else 0
        max_score: float = max([grasp.path_score for grasp in valid_grasps]) if len(valid_grasps) else 0
        
        # creates a GraspNetGroup
        gg: GraspNetGroup = GraspNetGroup()
        for hts_grasp in self._grasps:
            gg.add(hts_grasp.get_grasp_object())
        
        # converts to o3d geometry list
        grippers: list[Any] = gg.to_open3d_geometry_list()

        # paint grasps according to their grasp colour
        for ind, g in enumerate(grippers):
            if not self._grasps[ind].is_valid(): # paint black if invalid
                color: npt.NDArray[np.float64] = np.array([[0.0], [0.0], [0.0]], dtype=np.float64)
                g.paint_uniform_color(color)
            elif min_score != max_score: # otherwise paint it according to the path score
                color: npt.NDArray[np.float64] = np.array(cm.RdYlGn_r((self._grasps[ind].path_score - min_score)/(max_score - min_score))[:3], dtype=np.float64)
                g.paint_uniform_color(color)

        # create the origin frmae
        origin_frame: Any = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=origin_position)

        # visualise
        o3d.visualization.draw_geometries([*grippers, cloud, origin_frame], window_name=description)
