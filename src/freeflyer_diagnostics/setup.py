from setuptools import find_packages, setup

package_name = 'freeflyer_diagnostics'
node_names = [
    'force_velocity_live_plot',
    'error_plotter',
    'dual_error_plotter',
    ]

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
    maintainer_email='tonithetutor@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={'console_scripts': [f'{name} = {package_name}.{name}:main' for name in node_names]},
)
