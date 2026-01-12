from setuptools import setup
import os
from glob import glob

package_name = 'motion_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Motion control and grasp execution for OpenArm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal = motion_control.minimal:main',
            'motion_planner = motion_control.motion_planner:main',
            'gripper_controller = motion_control.gripper_controller:main',
            'grasp_state_machine = motion_control.grasp_state_machine:main',
            'fk_test = motion_control.fk_test:main',
            'auto_grasp_coordinator = motion_control.auto_grasp_coordinator:main',
            'ee_position_test = motion_control.ee_position_test:main',
        ],
    },
)
