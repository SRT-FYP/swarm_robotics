import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fyp/Desktop/swarm_repo/robot/ros2ws/install/hardware_software'
