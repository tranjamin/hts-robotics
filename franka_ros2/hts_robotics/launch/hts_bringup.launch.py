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
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node

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
            'ros2_control': 'true',
            'gazebo': gazebo_str,
            'ee_id': franka_hand_str,
            'use_fake_hardware': fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
            'robot_ip': robot_ip_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    return robot_description

def create_nodes(context: LaunchContext, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace):
    robot_description = get_robot_description(context, arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo)
    robot_description_semantic = get_robot_semantics(context, arm_id, load_gripper)
    robot_kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_kinematics_yaml,
        ],
    )

    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    namespace=namespace,
    parameters=[
        {'source_list': ['joint_states'],
            'rate': 30}],
    )

    hts_robotics_node = Node(
            package='hts_robotics',
            executable='hts_node',
            name='hts_node',
            output='screen',
            namespace=namespace,
        )


    return [robot_state_publisher, move_group_node, joint_state_publisher, hts_robotics_node]


def generate_launch_description():
    # parameter names for the launch file
    load_gripper_parameter_name = 'load_gripper'
    franka_hand_parameter_name = 'franka_hand'
    arm_id_parameter_name = 'arm_id'
    namespace_parameter_name = 'namespace'
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'fake_sensor_commands'
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
    opaque_nodes = OpaqueFunction(
        function = create_nodes,
        args=[arm_id, load_gripper, franka_hand, robot_ip, use_fake_hardware, fake_sensor_commands, use_gazebo, namespace]
    )

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
        output='screen',
        arguments=[
            "-topic", "/robot_description",
            "-name", "fr3",
        ],
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
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_hts_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'load_hts_controller'],
        output='screen'
    )

    load_dynamic_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'joint_position_example_controller'],
        output='screen'
    )

    controllers = [load_joint_state_broadcaster, load_hts_controller, load_dynamic_controller]

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        robot_ip_launch_argument,
        use_fake_hardware_launch_argument,
        fake_sensor_commands_launch_argument,
        use_gazebo_launch_argument,

        opaque_nodes,

        gazebo_empty_world,
        rviz,
        # moveit_launch,
        spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=controllers,
                )
        ),


    ])
