from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'pct_art_local_navigation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'config', 'local'),
            glob('config/local/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'config', 'unitree'),
            glob('config/unitree/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xuqiang656',
    maintainer_email='xuqiang656@example.com',
    description='Coordinate PCT global paths with ROG local planning.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pct_art_coordinator = pct_art_local_navigation.coordinator_node:main',
            (
                'pct_global_path_follow_coordinator = '
                'pct_art_local_navigation.global_path_follow_coordinator_node:main'
            ),
        ],
    },
)
