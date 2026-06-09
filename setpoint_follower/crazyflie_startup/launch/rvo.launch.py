import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.utilities import perform_substitutions

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


##############################################################################################################
# get drones indices and positions
##############################################################################################################

def get_drones_from_config(config_path):
    """
    Load drone positions from a ROS 2 parameter YAML file.

    Expected structure:
      /**:
        ros__parameters:
          agent_1:
            pos: [x, y, z]
            radio: 1
          agent_2:
            pos: [x, y, z]
            radio: 1
    """

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    if "/**" not in cfg:
        raise RuntimeError("Missing '/**' root key in config")

    params = cfg["/**"].get("ros__parameters", {})
    if params is None:
        raise RuntimeError("Missing ros__parameters in config")

    agents = {}
    for key, value in params.items():
        if key.startswith("agent_"):
            agent_id = int(key.split("_")[1])
            agents[agent_id] = value

    if not agents:
        raise RuntimeError("No agents found in config file")

    return agents


##############################################################################################################
# Runtime launch setup (THIS is where file access happens)
##############################################################################################################

def launch_setup(context, *args, **kwargs):

    # Resolve mission file path
    config_path = perform_substitutions(context, [MISSION_CONFIG])

    print(f"[launch] Loading mission file: {config_path}")

    drones = get_drones_from_config(config_path)
    drone_indices = sorted(drones.keys())

    backend_value = perform_substitutions(context, [BACKEND])
    use_sim_time_value = (backend_value == "sim")

    nodes = []

    # Manager node

    print(drone_indices)
    nodes.append(
        Node(
            package='rvo',
            executable='manager',
            name='manager_node',
            output='screen',
            parameters=[
                MISSION_CONFIG,
                {'AGENTS_INDICES': drone_indices},
                {'use_sim_time': use_sim_time_value},
                {'backend': backend_value},
            ]
        )
    )

    # Agent node
    nodes.append(
        Node(
            package='rvo',
            executable='agent_RVO',
            output='screen',
            parameters=[
                MISSION_CONFIG,
                {'AGENTS_INDICES': drone_indices},
                {'use_sim_time': use_sim_time_value},
                {'backend': backend_value},
            ]
        )
    )

    config_file = os.path.join(
        get_package_share_directory('crazyflie_ros2_setpoint_follower'),
        'config',
        'crazyflie.rviz'
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_file],
            condition=IfCondition(WITH_RVIZ)
    )

    nodes.append(rviz_node)

    return nodes


##############################################################################################################
# Main launch description
##############################################################################################################

def generate_launch_description():

    # ------------------------
    # Launch arguments
    # ------------------------

    with_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch RViz'
    )

    mission_yaml = DeclareLaunchArgument(
        'mission_yaml',
        default_value='config.yaml',
        description='Mission YAML file (relative to config/missions)'
    )

    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='sim',
        choices=['sim', 'hardware'],
        description='Backend selection'
    )

    # ------------------------
    # Launch configurations
    # ------------------------

    global BACKEND
    global MISSION_CONFIG
    global WITH_RVIZ

    BACKEND = LaunchConfiguration('backend')
    MISSION_CONFIG = PathJoinSubstitution([
        FindPackageShare('crazyflie_ros2_setpoint_follower'),
        'config',
        'missions',
        LaunchConfiguration('mission_yaml')
    ])

    WITH_RVIZ = LaunchConfiguration('rviz')

    # ------------------------
    # Crazyflie simulation (only for sim)
    # ------------------------

    pkg_project = get_package_share_directory(
        'crazyflie_ros2_setpoint_follower'
    )

    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'rvo_simu.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", BACKEND, "' == 'sim'"])
        )
    )

    crazyflie_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'crazyflie_real.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", BACKEND, "' == 'hardware'"])
        )
    )

    # ------------------------
    # Runtime node generation
    # ------------------------

    launch_setup_action = OpaqueFunction(
        function=launch_setup
    )

    # ------------------------
    # Final launch description
    # ------------------------

    return LaunchDescription([
        with_rviz,
        mission_yaml,
        backend_arg,
        crazyflie_simulation,
        crazyflie_real,
        launch_setup_action,
    ])
