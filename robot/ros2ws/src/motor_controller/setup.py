from setuptools import find_packages, setup

package_name = 'motor_controller'

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
    maintainer='fyp',
    maintainer_email='fyp@todo.todo',
    description='ROS2 package for motor control using GPIOZero',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = motor_controller.motor_driver:main',
            'wheel_encoder = motor_controller.wheel_encoder:main',
            'wheel_encoder_for_fusion = motor_controller.wheel_encoder_for_fusion:main',
        ],
    },
)
