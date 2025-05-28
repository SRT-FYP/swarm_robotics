from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'leader_follower'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),
         glob('launch/*launch.[pxy][yma]')),
         (os.path.join('share',package_name,'params/'),
         glob('params/*.yaml')),
        (os.path.join('share',package_name,'turtlebot3_models/turtlebot3_burger/'),
         glob('turtlebot3_models/turtlebot3_burger/*.*')),
        (os.path.join('share',package_name,'turtlebot3_models/turtlebot3_waffle/'),
         glob('turtlebot3_models/turtlebot3_waffle/*.*')),
         (os.path.join('share',package_name,'turtlebot3_models/turtlebot3_world/'),
         glob('turtlebot3_models/turtlebot3_world/*.*')),
         (os.path.join('share',package_name,'turtlebot3_models/turtlebot3_world/meshes'),
         glob('turtlebot3_models/turtlebot3_world/meshes/*.*')),
         (os.path.join('share',package_name,'urdf/'),
         glob('urdf/*.*')),
         (os.path.join('share',package_name,'models/turtlebot3_model/meshes/'),
         glob('models/turtlebot3_model/meshes/*.dae')),
         (os.path.join('share',package_name,'models/turtlebot3_model/'),
         glob('models/turtlebot3_model/*.*')),
         (os.path.join('share',package_name,'models/turtlebot3_world/meshes/'),
         glob('models/turtlebot3_world/meshes/*.dae')),
         (os.path.join('share',package_name,'models/turtlebot3_world/'),
         glob('models/turtlebot3_world/*.*')),
         (os.path.join('share',package_name,'models/turtlebot3_house/materials/textures/'),
         glob('models/turtlebot3_house/materials/textures/*.*')),
         (os.path.join('share',package_name,'models/turtlebot3_house/'),
         glob('models/turtlebot3_house/*.*')),
         (os.path.join('share',package_name,'worlds/'),
         glob('worlds/*.*')),
         (os.path.join('share',package_name,'config/'),
         glob('config/*.*')),
        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghina-debian',
    maintainer_email='ghina-debian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_node = leader_follower.robot_inter_communication:main',
            'odom_init = leader_follower.set_initial_odom:main',
            'tf_odom_init = leader_follower.set_init_odom_tf_frame:main',
        ],
    },
)
