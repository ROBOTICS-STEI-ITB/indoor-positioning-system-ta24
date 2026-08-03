"""Launch IPS pipeline — ALL compute nodes in C++.

C++ nodes:
    - clock_sync_node
    - position_solver_node      (algorithm='chan', default)  OR
    - wolf_position_node        (algorithm='wolf')
    - bias_compensator_node
    - imu_processor_node        (IMU Tingkat 1 — selalu jalan)
    - differentiator_node       (dipindah dari Python ke C++)

Python nodes (I/O only):
    - udp_gateway, calibration_service, state_aggregator, recorder

Default usage:
    ros2 launch ips_bringup ips_system_cpp.launch.py

Switch to WoLF-EKF-CA estimator:
    ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=wolf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)
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
    algorithm_arg = DeclareLaunchArgument(
        'algorithm', default_value='chan',
        description="Position estimator: 'chan' (pipeline lama, default) or "
                    "'wolf' (WoLF-EKF-CA, Duran-Martin et al. 2024)"
    )

    udp_port    = LaunchConfiguration('udp_port')
    log_path    = LaunchConfiguration('log_path')
    output_rate = LaunchConfiguration('output_rate_hz')
    record_dir  = LaunchConfiguration('record_dir')
    bias_yaml   = LaunchConfiguration('bias_yaml')
    algorithm   = LaunchConfiguration('algorithm')

    # ===== Python I/O nodes =====
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

    # ===== C++ compute nodes =====
    clock_sync = Node(
        package='ips_nodes_cpp', executable='clock_sync_node',
        name='clock_sync', output='screen',
        parameters=[system_yaml],
    )
    position_solver = Node(
        package='ips_nodes_cpp', executable='position_solver_node',
        name='position_solver', output='screen',
        parameters=[system_yaml],
        condition=IfCondition(PythonExpression(["'", algorithm, "' == 'chan'"])),
    )
    wolf_position = Node(
        package='ips_nodes_cpp', executable='wolf_position_node',
        name='wolf_position', output='screen',
        parameters=[system_yaml],
        condition=IfCondition(PythonExpression(["'", algorithm, "' == 'wolf'"])),
    )
    bias_compensator = Node(
        package='ips_nodes_cpp', executable='bias_compensator_node',
        name='bias_compensator', output='screen',
        parameters=[system_yaml, {'bias_yaml_path': bias_yaml}],
    )

    # ===== IMU compute nodes (C++) — selalu jalan (Tingkat 1) =====
    imu_processor = Node(
        package='ips_nodes_cpp', executable='imu_processor_node',
        name='imu_processor', output='screen',
        parameters=[system_yaml],
    )
    differentiator = Node(
        package='ips_nodes_cpp', executable='differentiator_node',
        name='differentiator', output='screen',
        parameters=[system_yaml],
    )

    return LaunchDescription([
        udp_port_arg, log_path_arg, output_rate_arg, record_dir_arg,
        bias_yaml_arg, algorithm_arg,
        calibration,
        udp_gateway,
        clock_sync,           # C++
        position_solver,      # C++ — active when algorithm='chan' (default)
        wolf_position,        # C++ — active when algorithm='wolf'
        bias_compensator,     # C++
        imu_processor,        # C++ — IMU Tingkat 1 (selalu jalan)
        differentiator,       # C++ — dipindah dari Python
        state_aggregator,
        recorder,
    ])
