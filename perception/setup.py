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
        # 下面这一行就是刚才容易出错的地方，现在已经修复了
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member B',
    maintainer_email='member_b@example.com',
    description='Perception package for OpenArm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注册视觉节点
            'vision_node = perception.object_detector:main',
            # 注册标定节点 (如果有)
            'calibration_node = calibration.calibration_script:main',
        ],
    },
)
