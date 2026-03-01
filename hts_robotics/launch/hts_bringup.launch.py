import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# SET TO FALSE FOR PERCEPTION PIPELINE, OR MAKE REALSENSE_CAMERA ALSO USE SIM TIME
USE_SIM_TIME = True

moveit_config = (
    MoveItConfigsBuilder("hts")
    .robot_description(
        file_path="config/fr3.urdf.xacro",
        mappings={
            "ros2_control_hardware_type": LaunchConfiguration(
                "ros2_control_hardware_type"
            )
        },
    )
    .robot_description_semantic(file_path="config/fr3.srdf")
    .planning_scene_monitor(
        publish_robot_description=True, publish_robot_description_semantic=True
    )
    .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    .planning_pipelines(
        pipelines=["stomp", "ompl", "chomp", "pilz_industrial_motion_planner"]
    )
    .to_moveit_configs()
)

def get_robot_description(context: LaunchContext, launch_configurations):
    subs = lambda x : context.perform_substitution(launch_configurations[x].get('launch_config'))
    arm_id_str = subs('arm_id')
    load_gripper_str = subs('load_gripper')
    franka_hand_str = subs('franka_hand')
    robot_ip_str = subs('robot_ip')
    fake_hardware_str = subs('use_fake_hardware')
    fake_sensor_commands_str = subs('fake_sensor_commands')
    gazebo_str = subs('gazebo')
    camera_str = subs('use_camera')

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
    subs = lambda x : context.perform_substitution(launch_configurations[x].get('launch_config'))
    arm_id_str = subs('arm_id')
    load_gripper_str = subs('load_gripper')
    camera_str = subs('use_camera')
    namespace_str = subs('namespace')

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
    rviz_base = os.path.join(get_package_share_directory('hts_robotics'), 'config')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    return rviz_full_config

def create_hts_node(context: LaunchContext, launch_configurations):
    robot_description = get_robot_description(context, launch_configurations)
    robot_description_semantic = get_robot_semantics(context, launch_configurations)
    namespace_str = context.perform_substitution(launch_configurations['namespace'].get('launch_config'))
    objects_yaml = load_yaml("hts_robotics", "config/target_objects.yaml")

    hts_node = Node(
        package='hts_robotics',
        executable='hts_node',
        name='hts_node',
        output='screen',
        namespace=namespace_str,
        parameters=[
            objects_yaml,
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
    namespace_str = context.perform_substitution(launch_configurations['namespace'].get('launch_config'))

    moveit_simple_controllers_yaml = load_yaml('hts_robotics', 'config/simple_controllers.yaml')
    sensors_yaml = load_yaml("hts_moveit_config", "config/sensors_realsense_pointcloud.yaml")
    general_config = load_yaml("hts_robotics", "config/config.yaml")

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace_str,
        parameters=[
            general_config,
            # sensors_yaml,
            robot_description,
            robot_description_semantic,

            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.pilz_cartesian_limits,
            moveit_config.trajectory_execution,
            moveit_config.robot_description_kinematics,
            moveit_controllers,
            load_yaml("hts_moveit_config", "config/planning_parameters.yaml"),
            planning_scene_monitor_parameters,
            {"use_sim_time": USE_SIM_TIME},
        ],
        arguments=[
            '--ros-args', '--log-level', 'debug'
        ]
    )

    return [move_group_node]

def create_rviz_node(context: LaunchContext, launch_configurations):
    robot_description = get_robot_description(context, launch_configurations)
    robot_description_semantic = get_robot_semantics(context, launch_configurations)
    namespace_str = context.perform_substitution(launch_configurations['namespace'].get('launch_config'))

    rviz_full_config = get_rviz_config()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace_str,
        output='log',
        arguments=[
            '-d', rviz_full_config,
            '--ros-args', '--log-level', 'info'
            ],
        parameters=[
            robot_description,
            robot_description_semantic,

            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.pilz_cartesian_limits,
            moveit_config.trajectory_execution,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": USE_SIM_TIME}
        ],
    )

    return [rviz_node]

def create_publisher_node(context: LaunchContext, launch_configurations):
    robot_description = get_robot_description(context, launch_configurations)    
    namespace_str = context.perform_substitution(launch_configurations['namespace'].get('launch_config'))


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

def include_gripper_launch(context: LaunchContext, launch_configurations):
    namespace_str = context.perform_substitution(launch_configurations['namespace'].get('launch_config'))
    load_gripper_param = context.perform_substitution(launch_configurations['load_gripper'].get('launch_config'))
    robot_ip_str = context.perform_substitution(launch_configurations['robot_ip'].get('launch_config'))
    fake_hardware_param = context.perform_substitution(launch_configurations['use_fake_hardware'].get('launch_config'))

    gripper_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={
                'namespace': namespace_str,
                'robot_ip': robot_ip_str,
                'use_fake_hardware': fake_hardware_param,
            }.items(),
            condition=IfCondition(load_gripper_param),
        )
    
    return [gripper_launch_file]


def define_parameters():
    params_yaml = load_yaml('hts_robotics', 'config/launch_params.yaml')
    params_dict = {}
    for p in params_yaml:
        param_name = list(p.keys())[0]
        param_options = list(p.values())[0]

        launch_config = LaunchConfiguration(param_name)
        launch_argument = DeclareLaunchArgument(
            param_name,
            default_value=str(param_options.get('default_value')),
            description=str(param_options.get('description', 'no description provided'))
        )

        params_dict[param_name] = {
            'name': param_name, 
            'launch_config': launch_config, 
            'launch_argument': launch_argument
            }
    
    return params_dict


def generate_launch_description():
    print("Generating launch description...")

    # parameters for the launch file
    print("Defining launch configuration for each parameter...")
    launch_params = define_parameters()

   # Gazebo Sim
    print("Defining Gazebo...")
    # os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.path.dirname(get_package_share_directory('franka_description')),
            os.path.dirname(get_package_share_directory('hts_robotics')),
        ])
    print(os.environ.get('GZ_SIM_RESOURCE_PATH'))

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
        namespace='',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    object_spawns = []
    targets_config_file = load_yaml("hts_robotics", "config/target_objects.yaml")
    if targets_config_file is not None:
        for obj in targets_config_file['objects']:
            obj_params = targets_config_file[obj]
            object_spawns.append(
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-file', os.path.join(get_package_share_directory("hts_robotics"), "objects", obj_params['geometry_file']),
                        '-name', 'target_' + str(obj_params['object_id']),
                        '-x', str(float(obj_params.get('x', 0))),
                        '-y', str(float(obj_params.get('y', 0))),
                        '-z', str(float(obj_params.get('z', 0))),
                        '-P', str(float(obj_params.get('pitch', 0))),
                        '-R', str(float(obj_params.get('roll', 0))),
                        '-Y', str(float(obj_params.get('yaw', 0))),
                        '-ros'
                    ],
                    output='screen'
                )
            )

    joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace='',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='',
        arguments=['fr3_arm_controller'],
        output='screen',
    )
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='',
        arguments=['gripper_position_controller'],
        output='screen',
    )

    # Get robot description
    moveit_node = OpaqueFunction(
        function = create_moveit_node,
        args=[launch_params]
    )

    rviz_node = OpaqueFunction(
        function = create_rviz_node,
        args=[launch_params]
    )

    hts_node = OpaqueFunction(
        function = create_hts_node,
        args=[launch_params]
    )

    state_publisher_node = OpaqueFunction(
        function = create_publisher_node,
        args=[launch_params]
    )

    gripper_launch = OpaqueFunction(
        function = include_gripper_launch,
        args=[launch_params]
    )

    joint_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='',
        parameters=[
            {'source_list': ['joint_states'], 'rate': 30},
            {"use_sim_time": USE_SIM_TIME},
        ],
    )

    topic_bridges = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/ros_gz/model/pose@ros_gz_interfaces/msg/Entity@gz.msgs.Entity",
            "/world/empty/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            
            "/camera_sim/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera_sim/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_sim/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_sim/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            '--ros-args', '--log-level', 'error',
        ],
        parameters = [
            {'use_sim_time': USE_SIM_TIME}
        ],
        output="log",
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

    anygrasp_node = Node(
        package='hts_anygrasp',
        executable='anygrasp_node',
        output='screen',
        parameters=[
            load_yaml('hts_robotics', 'config/anygrasp_params.yaml')
        ]
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name="octomap_sim",
        output="log",
        parameters=[{
            'frame_id': 'world',
            'base_frame_id': 'world',

            'resolution': 0.005,
            'use_sim_time': True,

            # 'occupancy_min_z': 0.01,
            'occupancy_max_z': 0.5,

            'filter_ground_plane': True,
            'filter_speckles': True,
            # 'ground_filter.angle':
            'ground_filter.distance': 0.01,
            'ground_filter.plane_distance': 0.01,

            'sensor_model.hit': 0.9,
            'sensor_model.miss': 0.03,
            'sensor_model.max_range': 1.0,
        }],
        remappings=[
            ('/cloud_in', '/camera_sim/points')  # your canonical cloud topic
        ],
        arguments=[
            '--ros-args', '--log-level', 'error',
        ]
    )
    
    camera_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_camera_static_tf",
        arguments=[
        "--frame-id", "camera_depth_frame",
        "--child-frame-id", "fr3/fr3_link7/custom_camera_rgbd",
        '--ros-args', '--log-level', 'error',
        ],
        parameters=[
            { 'use_sim_time': True}
        ],
        output="log"
    )

    all_launch_arguments = [x.get('launch_argument') for (_, x) in launch_params.items()]

    return LaunchDescription(all_launch_arguments + [
        gazebo_empty_world,
        # gripper_launch,
        # realsense_node,
        anygrasp_node,
        camera_transform,
        octomap_node,


        TimerAction(
            period=5.0,
            actions=[topic_bridges]
        ),
        
        TimerAction(
            period=10.0,
            actions=[state_publisher_node]
        ),

        TimerAction(
            period=15.0,
            actions=[spawn] + object_spawns
        ),

        TimerAction(
            period=20.0,
            actions=[joint_broadcaster, gripper_controller, arm_controller]
        ),

        TimerAction(
            period=20.0,
            actions=[joint_publisher_node]
        ),

        TimerAction(
            period=30.0,
            actions=[moveit_node, rviz_node, hts_node]
        ),
    ])
