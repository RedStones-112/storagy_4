from setuptools import find_packages, setup
import os
import glob

package_name = 'control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param',
        glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rds',
    maintainer_email='wogur21c@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_planner = control_node.robot_planner:main',
            'aruco_follower = control_node.aruco_follower:main'
        ],
    },
)
