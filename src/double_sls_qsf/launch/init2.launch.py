# gpt improved version of the original file
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Launch Gazebo with PX4 SITL and ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = os.path.join(HOME, 'tmp/px4_run_dir')
    PX4_AIRFRAMES_DIR = os.path.join(HOME, 'PX4-Autopilot/build/px4_sitl_default/etc/init.d/airframes')
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # World path
    world_path = os.path.join(HOME, 'PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world')

    # Model path
    setup_package_path = get_package_share_directory('setup')
    model_path = os.path.join(setup_package_path, 'models/px4vision_sls/px4vision_sls.sdf')

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        # Ensure Gazebo finds both PX4 and custom models
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(HOME, 'PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models') + 
            ':' + os.path.join(setup_package_path, 'models') +
            ':$GAZEBO_MODEL_PATH'  # Appending instead of overwriting
        ),

        # PX4 plugins
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', HOME + '/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic'),

        # Set Drone ID and Estimator Type
        DeclareLaunchArgument('drone_id', default_value='1', description='PX4 Drone ID'),
        SetEnvironmentVariable('PX4_SIM_DRONE_ID', LaunchConfiguration('drone_id')),
        SetEnvironmentVariable('PX4_MAV_SYS_ID', LaunchConfiguration('drone_id')),
        DeclareLaunchArgument('est', default_value='ekf2', description='PX4 Estimator Type'),
        SetEnvironmentVariable('PX4_ESTIMATOR', LaunchConfiguration('est')),

        # Fix missing airframe files in PX4 run directory
        ExecuteProcess(
            cmd=['cp', '-r', PX4_AIRFRAMES_DIR, os.path.join(PX4_RUN_DIR, 'etc/init.d-posix/')],
            output='screen'
        ),

        # Declare launch arguments for model positioning
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('model', default_value=model_path),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('initial_sim_time', default_value=''),

        # Launch Gazebo server and client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'verbose': 'true',
                'pause': LaunchConfiguration('paused'),
                'initial_sim_time': LaunchConfiguration('initial_sim_time')
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        ),

        # spawning the model
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'model',
                        '--spawn-file', LaunchConfiguration('model'),
                        '--model-name', 'px4vision_sls',
                        '-x', LaunchConfiguration('x'),
                        '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'),
                        '-R', LaunchConfiguration('R'),
                        '-P', LaunchConfiguration('P'),
                        '-Y', LaunchConfiguration('Y'),
                    ],
                    output='screen'
                )
            ]
        ),

        # Start PX4 SITL
        ExecuteProcess(
            cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_default/bin/px4',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s', HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
            ],
            cwd=PX4_RUN_DIR,
            shell=True,
            prefix="xterm -hold -e",
            output='screen'
        ),

        # Delay Micro XRCE-DDS Agent startup (avoid race conditions)
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                    output='screen'
                )
            ]
        ),
    ])
