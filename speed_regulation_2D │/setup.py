from setuptools import setup

package_name = 'simulation_2d_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'simulation_node = simulation_2d_ros.simulation_node:main',
            'bridge_ros = simulation_2d_ros.bridge_ros:main',
        ],
    },
)
