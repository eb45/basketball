from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'basketball_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Basketball scene, overhead camera, and ball detection for the Kinova Gen3 Lite',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'basketball_scene = basketball_moveit.scripts.basketball_scene:main',
            'spawn_overhead_camera = basketball_moveit.scripts.spawn_overhead_camera:main',
            'ball_detector = basketball_moveit.scripts.ball_detector:main',
            'check_ball = basketball_moveit.scripts.check_ball:main',
            'pick_ball = basketball_moveit.scripts.pick_ball:main',
            'dunk_ball = basketball_moveit.scripts.dunk_ball:main',
        ],
    },
)