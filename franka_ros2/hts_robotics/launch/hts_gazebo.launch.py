import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def get_robot_description_2(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

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
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]

def get_robot_description(context: LaunchContext, 
        arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    robot_ip_str = context.perform_substitution(robot_ip)
    fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    gazebo_str = context.perform_substitution(use_gazebo)

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
            # 'ros2_control': 'true',
            # 'gazebo': gazebo_str,
            'ee_id': franka_hand_str,
            # 'use_fake_hardware': fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
            'robot_ip': robot_ip_str
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

def get_robot_semantics(context: LaunchContext, arm_id, load_gripper):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, arm_id_str + '.srdf.xacro'
    )

    robot_description_semantic_config = xacro.process_file(
        franka_semantic_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str
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
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    return ompl_planning_pipeline_config

def create_moveit_nodes(context: LaunchContext, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace):
    robot_description = get_robot_description(context, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo)
    robot_description_semantic = get_robot_semantics(context, arm_id, load_gripper)
    robot_kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
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

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    ompl_planning_pipeline_config = get_ompl_config()
    rviz_full_config = get_rviz_config()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d', rviz_full_config,
            '--ros-args', '--log-level', 'warn'
            ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_kinematics_yaml,
        ],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
        arguments=[
            '--ros-args', '--log-level', 'warn'
        ]
    )

    return [move_group_node, rviz_node]

def create_control_nodes(context: LaunchContext, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace):
    robot_description = get_robot_description(context, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo)
    robot_description_semantic = get_robot_semantics(context, arm_id, load_gripper)
    robot_kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    ros2_controllers_path = os.path.join(
        get_package_share_directory('hts_robotics'),
        'config',
        'controllers.yaml',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # Load controllers
    load_controllers = []
    for controller in ['fr3_arm_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager',
                    PathJoinSubstitution([namespace, 'controller_manager'])
                ],
                output='screen'
            )
    )


    return [ros2_control_node] + load_controllers

def create_basic_nodes(context: LaunchContext, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace):
    robot_description = get_robot_description(context, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo)
    robot_description_semantic = get_robot_semantics(context, arm_id, load_gripper)
    robot_kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
        arguments=[
            '--ros-args', '--log-level', 'warn'
        ]
    )

    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    namespace=namespace,
    parameters=[
        {'source_list': ['joint_states'],
            'rate': 30}],
    arguments=[
            '--ros-args', '--log-level', 'warn'
        ]
    )

    hts_robotics_node = Node(
            package='hts_robotics',
            executable='hts_node',
            name='hts_node',
            output='screen',
            namespace=namespace,
        )

    return [joint_state_publisher, hts_robotics_node]

def generate_launch_description():
    # parameter names for the launch file
    load_gripper_parameter_name = 'load_gripper'
    franka_hand_parameter_name = 'franka_hand'
    arm_id_parameter_name = 'arm_id'
    namespace_parameter_name = 'namespace'
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_gazebo_parameter_name = 'gazebo'

    # parameters for the launch file
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    franka_hand = LaunchConfiguration(franka_hand_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_gazebo = LaunchConfiguration(use_gazebo_parameter_name)

    # define launch arguments
    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='false',
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
            default_value='false',
            description='true/false to pass gazebo=true to URDF')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description_2,
        args=[arm_id, load_gripper, franka_hand])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             namespace=namespace,
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )
    load_fr3_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'fr3_arm_controller'],
        output='screen'
    )

    # Get robot description
    opaque_nodes_moveit = OpaqueFunction(
        function = create_moveit_nodes,
        args=[arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace]
    )
    opaque_nodes_control = OpaqueFunction(
        function = create_control_nodes,
        args=[arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace]
    )
    opaque_nodes_basic = OpaqueFunction(
        function = create_basic_nodes,
        args=[arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace]
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

        gazebo_empty_world,
        robot_state_publisher,
        opaque_nodes_basic,
        opaque_nodes_moveit,
        rviz,
        spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster, load_fr3_arm_controller],
                )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
    ])
