from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waiter_b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vj',
    maintainer_email='miracleyt404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arduino_bridge = waiter_b.arduino_bridge:main',
            'lidar_filter = waiter_b.lidar_filter:main',
            'teleop = waiter_b.teleop:main',
            'dead_reckon = waiter_b.dead_reckon:main',
        ],
    },
)
