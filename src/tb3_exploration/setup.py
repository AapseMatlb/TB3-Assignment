from setuptools import setup
package_name = 'tb3_exploration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_explore.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_toolbox_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='SLAM + Nav2 launch and configs for TurtleBot3',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
