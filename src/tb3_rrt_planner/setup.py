from setuptools import setup
package_name = 'tb3_rrt_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','pyyaml','matplotlib','Pillow'],
    zip_safe=True,
    maintainer='Yashashwani Kashyap',
    maintainer_email='kashyapyashashwani@gmail.com',
    description='Custom RRT planner publishing nav_msgs/Path',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rrt_planner = tb3_rrt_planner.rrt_planner_node:main',
        ],
    },
)
