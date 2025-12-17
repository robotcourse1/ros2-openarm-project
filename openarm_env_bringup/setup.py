from setuptools import setup
from glob import glob
import os

package_name = 'openarm_env_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Bringup for OpenArm environment with table/fruits/camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_ros_bridge = openarm_env_bringup.mujoco_ros_bridge:main',
        ],
    },
)
