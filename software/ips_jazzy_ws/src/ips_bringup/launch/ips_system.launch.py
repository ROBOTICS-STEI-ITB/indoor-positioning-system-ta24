"""Launch IPS pipeline (Python I/O nodes + C++ bias_compensator).

This launch uses Python for I/O-only nodes (udp_gateway, recorder, etc.)
and Python compute nodes (clock_sync, position_solver). Bias_compensator
is C++ since user prefers all computations in C++.

For ALL-C++ compute (clock_sync + position_solver + bias_compensator),
use ips_system_cpp.launch.py.

Default usage:
    ros2 launch ips_bringup ips_system.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('ips_bringup')

    anchors_yaml = PathJoinSubstitution([pkg_share, 'config', 'anchors.yaml'])
    system_yaml  = PathJoinSubstitution([pkg_share, 'config', 'system.yaml'])

    udp_port_arg = DeclareLaunchArgument('udp_port', default_value='5555')
    log_path_arg = DeclareLaunchArgument('log_path', default_value='')
    output_rate_arg = DeclareLaunchArgument('output_rate_hz', default_value='10.0')
    record_dir_arg = DeclareLaunchArgument('record_dir', default_value='~/ips_logs')
    bias_yaml_arg = DeclareLaunchArgument(
        'bias_yaml', default_value='~/ips_jazzy_ws/bias.yaml',
        description='Path to persisted bias YAML'
    )

    udp_port    = LaunchConfiguration('udp_port')
    log_path    = LaunchConfiguration('log_path')
    output_rate = LaunchConfiguration('output_rate_hz')
    record_dir  = LaunchConfiguration('record_dir')
    bias_yaml   = LaunchConfiguration('bias_yaml')

    calibration = Node(
        package='ips_nodes', executable='calibration_service',
        name='calibration_service', output='screen',
        parameters=[{'anchors_yaml': anchors_yaml}],
    )
    udp_gateway = Node(
        package='ips_nodes', executable='udp_gateway',
        name='udp_gateway', output='screen',
        parameters=[system_yaml, {'udp_port': udp_port}],
    )
    clock_sync = Node(
        package='ips_nodes', executable='clock_sync',
        name='clock_sync', output='screen',
        parameters=[system_yaml],
    )
    position_solver = Node(
        package='ips_nodes', executable='position_solver',
        name='position_solver', output='screen',
        parameters=[system_yaml],
    )

    # bias_compensator: C++ (all calibration math in C++)
    bias_compensator = Node(
        package='ips_nodes_cpp', executable='bias_compensator_node',
        name='bias_compensator', output='screen',
        parameters=[system_yaml, {'bias_yaml_path': bias_yaml}],
    )

    differentiator = Node(
        package='ips_nodes', executable='differentiator',
        name='differentiator', output='screen',
        parameters=[system_yaml],
    )
    state_aggregator = Node(
        package='ips_nodes', executable='state_aggregator',
        name='state_aggregator', output='screen',
        parameters=[
            system_yaml,
            {'output_rate_hz': output_rate, 'log_file_path': log_path},
        ],
    )
    recorder = Node(
        package='ips_nodes', executable='recorder',
        name='recorder', output='screen',
        parameters=[{'base_dir': record_dir}],
    )

    return LaunchDescription([
        udp_port_arg, log_path_arg, output_rate_arg, record_dir_arg, bias_yaml_arg,
        calibration,
        udp_gateway,
        clock_sync,
        position_solver,
        bias_compensator,    # C++
        differentiator,
        state_aggregator,
        recorder,
    ])
