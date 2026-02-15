from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'combined_pose_system'

entry_points = {'console_scripts': []}
nodes = ['gst_pose_cam', 'vision_yaw', 'starfield_sync_node']
for node in nodes: 
    entry_points['console_scripts'].append( f'{node} = combined_pose_system.nodes.{node}:main')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toni',
    maintainer_email='toni@todo.todo',
    description='Combined Marvelmind + vision pose system',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points=entry_points,

)
