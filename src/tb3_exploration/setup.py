from setuptools import setup
package_name = 'tb3_exploration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # ensure tb3_exploration/__init__.py exists
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/slam_explore.launch.py',
            'launch/view_map.launch.py',        # <- install the RViz launcher too
        ]),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_params.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/tb3_mapping.rviz',            # <- install the RViz config
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yashashwani Kashyap',
    maintainer_email='kashyapyashashwani@gmail.com',
    description='SLAM + Nav2 launch and configs for TurtleBot3',
    license='MIT',
    entry_points={
        'console_scripts': [
            # no console scripts for this package
        ],
    },
)
