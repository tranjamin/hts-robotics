import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# SET TO FALSE FOR PERCEPTION PIPELINE, OR MAKE REALSENSE_CAMERA ALSO USE SIM TIME
USE_SIM_TIME = False

load_gripper_parameter_name = 'load_gripper'
franka_hand_parameter_name = 'franka_hand'
arm_id_parameter_name = 'arm_id'
namespace_parameter_name = 'namespace'
robot_ip_parameter_name = 'robot_ip'
use_fake_hardware_parameter_name = 'use_fake_hardware'
fake_sensor_commands_parameter_name = 'fake_sensor_commands'
use_gazebo_parameter_name = 'gazebo'
use_camera_parameter_name = 'use_camera'

def get_robot_description(context: LaunchContext, launch_configurations):
    subs = lambda x : context.perform_substitution(launch_configurations.get(x))
    arm_id_str = subs(arm_id_parameter_name)
    load_gripper_str = subs(load_gripper_parameter_name)
    franka_hand_str = subs(franka_hand_parameter_name)
    robot_ip_str = subs(robot_ip_parameter_name)
    fake_hardware_str = subs(use_fake_hardware_parameter_name)
    fake_sensor_commands_str = subs(fake_sensor_commands_parameter_name)
    gazebo_str = subs(use_gazebo_parameter_name)
    camera_str = subs(use_camera_parameter_name)
    namespace_str = subs(namespace_parameter_name)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': gazebo_str,
            'gazebo_effort': gazebo_str,
            'ee_id': franka_hand_str,
            'use_fake_hardware': fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
            'robot_ip': robot_ip_str,
            'use_camera': camera_str,
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    return robot_description

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* Windows Error where available
        return None

def get_robot_semantics(context: LaunchContext, launch_configurations):
    subs = lambda x : context.perform_substitution(launch_configurations.get(x))
    arm_id_str = subs(arm_id_parameter_name)
    load_gripper_str = subs(load_gripper_parameter_name)
    camera_str = subs(use_camera_parameter_name)
    namespace_str = subs(namespace_parameter_name)

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, arm_id_str + '.srdf.xacro'
    )

    robot_description_semantic_config = xacro.process_file(
        franka_semantic_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'camera': camera_str,
        }
    )

    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config.toxml()}
    return robot_description_semantic

def get_rviz_config():
    rviz_base = os.path.join(get_package_share_directory('franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    return rviz_full_config

def get_ompl_config():
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'hts_robotics', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    return ompl_planning_pipeline_config

def create_hts_node(context: LaunchContext, launch_configurations):
    robot_description = get_robot_description(context, launch_configurations)
    robot_description_semantic = get_robot_semantics(context, launch_configurations)
    namespace_str = context.perform_substitution(launch_configurations.get(namespace_parameter_name))

    hts_node = Node(
        package='hts_robotics',
        executable='hts_node',
        name='hts_node',
        output='screen',
        namespace=namespace_str,
        parameters=[
            {"use_sim_time": USE_SIM_TIME},
            robot_description,
            robot_description_semantic,
        ],
        arguments=[
            '--ros-args', '--log-level', 'info'
        ]
    )

    return [hts_node]

def create_moveit_node(context: LaunchContext, launch_configurations):
    robot_description = get_robot_description(context, launch_configurations)
    robot_description_semantic = get_robot_semantics(context, launch_configurations)
    robot_kinematics_yaml = load_yaml('hts_robotics', 'config/kinematics.yaml')
    namespace_str = context.perform_substitution(launch_configurations.get(namespace_parameter_name))

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'hts_robotics', 'config/simple_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    sensors_yaml = load_yaml("hts_robotics", "config/sensors_kinect_pointcloud.yaml")

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        # 'sensors': sensors_yaml
    }

    octomap_config = {
        'octomap_frame': 'world',
        'octomap_resolution': 0.05,
        'max_range': 5.0
    }


    ompl_planning_pipeline_config = get_ompl_config()

    trajectory_config = load_yaml("hts_robotics", "config/trajectory_execution.yaml")

    print("Sensors YAML: ", sensors_yaml)
    print("Trajectory Config: ", trajectory_config)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace_str,
        parameters=[
            octomap_config,
            sensors_yaml,
            robot_description,
            robot_description_semantic,
            robot_kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            trajectory_config,
            {"use_sim_time": USE_SIM_TIME},
            {
                "goal_joint_tolerance": 0.01,
                "goal_position_tolerance": 0.01,
                "goal_orientation_tolerance": 0.01
            }
        ],
        arguments=[
            '--ros-args', '--log-level', 'error'
        ]
    )

    return [move_group_node]

def create_rviz_node(context: LaunchContext, launch_configurations):
    robot_description = get_robot_description(context, launch_configurations)
    robot_description_semantic = get_robot_semantics(context, launch_configurations)
    robot_kinematics_yaml = load_yaml('hts_robotics', 'config/kinematics.yaml')
    namespace_str = context.perform_substitution(launch_configurations.get(namespace_parameter_name))

    ompl_planning_pipeline_config = get_ompl_config()
    rviz_full_config = get_rviz_config()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace_str,
        output='log',
        arguments=[
            '-d', rviz_full_config,
            '--ros-args', '--log-level', 'error'
            ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_kinematics_yaml,
            {"use_sim_time": USE_SIM_TIME}
        ],
    )

    return [rviz_node]

def create_publisher_node(context: LaunchContext, launch_configurations):
    robot_description = get_robot_description(context, launch_configurations)    
    namespace_str = context.perform_substitution(launch_configurations.get(namespace_parameter_name))


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace=namespace_str,
        parameters=[
            robot_description,
            {"use_sim_time": USE_SIM_TIME},
            ],
        arguments=[
            '--ros-args', '--log-level', 'error'
        ]
    )

    return [robot_state_publisher]

def define_parameters():
    pass

def generate_launch_description():
    print("Generating launch description...")

    # parameters for the launch file
    print("Defining launch configuration for each parameter...")
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    franka_hand = LaunchConfiguration(franka_hand_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_gazebo = LaunchConfiguration(use_gazebo_parameter_name)
    use_camera = LaunchConfiguration(use_camera_parameter_name)

    launch_configurations = {
        load_gripper_parameter_name: load_gripper,
        franka_hand_parameter_name: franka_hand,
        arm_id_parameter_name: arm_id,
        namespace_parameter_name: namespace,
        robot_ip_parameter_name: robot_ip,
        use_fake_hardware_parameter_name: use_fake_hardware,
        fake_sensor_commands_parameter_name: fake_sensor_commands,
        use_gazebo_parameter_name: use_gazebo,
        use_camera_parameter_name: use_camera,
    }

    # define launch arguments
    print("Declaring launch arguments...")
    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_parameter_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3',
            description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
            namespace_parameter_name,
            default_value='',
            description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')
    robot_ip_launch_argument = DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value='0',
            description='the ip of the robot arm')
    use_fake_hardware_launch_argument = DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='true/false to use fake hardware')
    fake_sensor_commands_launch_argument = DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='unknown')
    use_gazebo_launch_argument = DeclareLaunchArgument(
            use_gazebo_parameter_name,
            default_value='true',
            description='true/false to pass gazebo=true to URDF')
    use_camera_launch_argument = DeclareLaunchArgument(
            use_camera_parameter_name,
            default_value='true',
            description='true/false to use the D435 camera'
    )

   # Gazebo Sim
    print("Defining Gazebo...")
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': 'empty.sdf -r', 
            'ros_clock_publisher': 'false',
            'ros_args': '--log-level warn',
            }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['fr3_arm_controller'],
        output='screen',
    )

    # Get robot description
    moveit_node = OpaqueFunction(
        function = create_moveit_node,
        args=[launch_configurations]
    )

    rviz_node = OpaqueFunction(
        function = create_rviz_node,
        args=[launch_configurations]
    )

    hts_node = OpaqueFunction(
        function = create_hts_node,
        args=[launch_configurations]
    )

    state_publisher_node = OpaqueFunction(
        function = create_publisher_node,
        args=[launch_configurations]
    )

    joint_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30},
                {"use_sim_time": USE_SIM_TIME},
                 
                 ],
        )

    clock_bridge = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # "/camera/color@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera/depth/color/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloud2",
            # "/camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"
        ],
        output="screen",
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
        ),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items()
    )


    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        robot_ip_launch_argument,
        use_fake_hardware_launch_argument,
        fake_sensor_commands_launch_argument,
        use_gazebo_launch_argument,
        use_camera_launch_argument,

        gazebo_empty_world,
        realsense_node,

        # TimerAction(
        #     period=5.0,
        #     actions=[camera_bridge]
        # ),

        TimerAction(
            period=5.0,
            actions=[camera_bridge]
        ),
        
        TimerAction(
            period=10.0,
            actions=[state_publisher_node]
        ),

        TimerAction(
            period=15.0,
            actions=[spawn]
        ),

        TimerAction(
            period=20.0,
            actions=[arm_controller, joint_broadcaster]
        ),

        TimerAction(
            period=20.0,
            actions=[joint_publisher_node]
        ),

        TimerAction(
            period=25.0,
            actions=[moveit_node, rviz_node, hts_node]
        ),
    ])
