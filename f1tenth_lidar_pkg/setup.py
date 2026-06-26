from setuptools import find_packages, setup

package_name = 'f1tenth_lidar_pkg'

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
    maintainer='shirley',
    maintainer_email='shirley@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'lidar_analysis = f1tenth_lidar_pkg.lidar_analysis_node:main',
		'simple_drive = f1tenth_lidar_pkg.simple_drive_node:main',
		'pure_pursuit = f1tenth_lidar_pkg.pure_pursuit_node:main',
		'follow_the_gap = f1tenth_lidar_pkg.follow_the_gap_node:main',
        'static_objects = f1tenth_lidar_pkg.add_static_objects:main',
        'opponent_driver = f1tenth_lidar_pkg.opponent_driver_node:main'		
        ],
    },
)
