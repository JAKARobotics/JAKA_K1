from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    script_dir = PathJoinSubstitution([FindPackageShare('jaka_k1_isaacsim'), 'scripts'])
    python_sh = PathJoinSubstitution([script_dir, 'python.sh'])

    usd_abs = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('usd_pkg')),
        LaunchConfiguration('usd_rel'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('usd_pkg', default_value='jaka_k1_description'),
        DeclareLaunchArgument('usd_rel', default_value='k1-de/urdf/jaka_k1/jaka_k1_moveit.usd'),
        ExecuteProcess(
            cmd=[
                python_sh, 'isaacsim_moveit.py'  
            ],
            additional_env={'K1_USD_PATH': usd_abs},
            output='screen',
        )
    ])
