from setuptools import setup

package_name = 'my_cool_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/myfile.launch.py']),
        ('share/' + package_name + '/maps', ['maps/10by10_maze.world_1.xml']),  # ✅ Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woeber',
    maintainer_email='woeber@todo.todo',
    description='My TurtleBot3 + Kalman launch',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
