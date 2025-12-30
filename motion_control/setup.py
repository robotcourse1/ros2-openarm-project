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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='member_c',
    maintainer_email='member_c@example.com',
    description='Motion planning and control package for openarm robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal = motion_control.minimal:main',
            'grasp_planner = motion_control.grasp_planner:main',
            'bimanual_grasp_planner = motion_control.bimanual_grasp_planner:main',
            'gripper_controller = motion_control.gripper_controller:main',
            'test_grasp = motion_control.test_grasp:main',
            'statistics_analyzer = motion_control.statistics_analyzer:main',
        ],
    },
)
