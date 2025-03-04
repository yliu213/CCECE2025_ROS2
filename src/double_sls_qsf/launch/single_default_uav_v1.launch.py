# general template for sitl initialization of a single uav in ros2
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Launch Gazebo with PX4 SITL and ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = os.path.join(HOME, 'tmp/px4_run_dir')
    PX4_AIRFRAMES_DIR = os.path.join(HOME, 'PX4-Autopilot/build/px4_sitl_default/etc/init.d/airframes')
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # Use PX4's default iris drone model
    world_path = os.path.join(HOME, 'PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world')
    model_path = os.path.join(HOME, 'PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf')

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        # Set environment variables for Gazebo and PX4
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', HOME + '/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'),
        SetEnvironmentVariable('PX4_SIM_MODEL', 'iris'),

        # Ensure PX4 finds the correct airframe
        SetEnvironmentVariable('PX4_SYS_AUTOSTART', '4001'),  # Autostart ID for iris
        SetEnvironmentVariable('PX4_SYS_AUTOCONFIG', '1'),

        # Copy airframe files to PX4 run directory (fixes missing model issue)
        ExecuteProcess(
            cmd=['cp', '-r', PX4_AIRFRAMES_DIR, os.path.join(PX4_RUN_DIR, 'etc/init.d-posix/')],
            output='screen'
        ),

        # Declare launch arguments
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('model', default_value=model_path),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),

        # Launch Gazebo server and client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={'world': LaunchConfiguration('world'), 'verbose': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        ),

        # Spawn PX4's default iris model
        ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model'),
                '--model-name', 'iris',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y'),
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),

        # Start PX4 SITL
        ExecuteProcess(
            cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_default/bin/px4',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS'
            ],
            cwd=PX4_RUN_DIR,
            output='screen'),

        # Start Micro XRCE-DDS Agent for ROS2 <--> PX4 communication
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'),
    ])
