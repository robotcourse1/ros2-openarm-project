import os
from glob import glob
from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 任务 2 的铺垫：安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member B',
    maintainer_email='member_b@example.com',
    description='Perception module for OpenArm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 任务 1：注册核心节点
            'object_detector = perception.object_detector:main',
            # 如果你有 minimal.py 也可以加，没有就注释掉
            # 'minimal = perception.minimal:main', 
        ],
    },
)
