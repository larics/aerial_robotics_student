from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aerial_robotics_tasks'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Aerial Robotics Tasks Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'height_ctl_node = aerial_robotics_tasks.task2_height_ctl:main',
            'yaw_ctl_node = aerial_robotics_tasks.task3_yaw_ctl:main',
            'mux_node = aerial_robotics_tasks.mux:main',
            'horizontal_ctl_node = aerial_robotics_tasks.task4_horizontal_ctl:main',
        ],
    },
)
