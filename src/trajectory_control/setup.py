from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trajectory_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kingkhan',
    maintainer_email='your-email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_publisher = trajectory_control.waypoint_publisher:main',
            'path_smoother = trajectory_control.path_smoother:main',
            'trajectory_generator = trajectory_control.trajectory_generator:main',
            'controller = trajectory_control.controller:main',
        ],
    },
)
