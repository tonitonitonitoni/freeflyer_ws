from setuptools import find_packages, setup

package_name = 'gst_camera_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toni',
    maintainer_email='toni@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_cam = gst_camera_streamer.arm_cam:main',
            'pose_cam = gst_camera_streamer.pose_cam:main'
        ],
    },
)
