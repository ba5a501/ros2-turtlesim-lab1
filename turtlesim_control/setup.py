import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlesim_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ziad Ahmed',
    maintainer_email='ziad@todo.todo',
    description='Turtlesim control: goToPose, square, reset service, speed parameters',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_action_server = turtlesim_control.pose_action_server:main',
            'square_action_server = turtlesim_control.square_action_server:main',
            'reset_service_node = turtlesim_control.reset_service_node:main',
        ],
    },
)
