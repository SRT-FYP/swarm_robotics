from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hardware_software'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),
         glob('launch/*.[pxy][yma]')),
        (os.path.join('share',package_name,'urdf/'),
         glob('urdf/*.*')),
         (os.path.join('share',package_name,'params/'),
         glob('params/*.*')),
        (os.path.join('share',package_name,'rviz/'),
         glob('rviz/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fyp',
    maintainer_email='fyp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_mqtt_encoder = hardware_software.mqtt_encoder:main',
            'ros2_mqtt_subscriber = hardware_software.mqtt_subscriber:main',
            'map_to_json = hardware_software.map_to_json:main',
        ],
    },
)
