from setuptools import setup

package_name = 'my_cool_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 👈 this must match the subfolder name
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/myfile.launch.py']),
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
