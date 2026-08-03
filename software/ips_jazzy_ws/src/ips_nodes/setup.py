from setuptools import find_packages, setup

package_name = 'ips_nodes'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='Karei',
    maintainer_email='karei@todo.local',
    description='ROS2 nodes for the UWB-based indoor positioning system (TA Karei).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_gateway = ips_nodes.udp_gateway_node:main',
            'clock_sync = ips_nodes.clock_sync_node:main',
            'position_solver = ips_nodes.position_solver_node:main',
            'differentiator = ips_nodes.differentiator_node:main',
            'state_aggregator = ips_nodes.state_aggregator_node:main',
            'latency_monitor = ips_nodes.latency_monitor_node:main',
            'calibration_service = ips_nodes.calibration_service_node:main',
            'recorder = ips_nodes.recorder_node:main',
            'bias_compensator = ips_nodes.bias_compensator_node:main',
        ],
    },
)
