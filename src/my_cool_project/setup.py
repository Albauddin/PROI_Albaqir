from setuptools import setup
import os
from glob import glob

package_name = 'my_cool_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ✅ Required marker for package indexing
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # Standard installation paths
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),    # If you have maps
        (os.path.join('share', package_name, 'config'), glob('config/*')), # If you have config

        # ('share/' + package_name + '/launch', [
        #     'launch/myfile.launch.py',
        #     'launch/myfile2.launch.py',
        #     'launch/start.launch.py',
        #     # ('share/my_cool_project/launch', glob('launch/*.launch.py'))
        # ]),
        # ('share/' + package_name + '/maps', [
        #     'maps/10by10_maze.world_1.xml'
        # ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woeber',
    maintainer_email='woeber@todo.todo',
    description='My TurtleBot3 + Kalman launch',
    license='MIT',
    entry_points={
        'console_scripts': [
            'initial_pose_publisher = my_cool_project.initial_pose_publisher:main',
            'run_demo = my_cool_project.run_demo:main',
        ],
    },
)
