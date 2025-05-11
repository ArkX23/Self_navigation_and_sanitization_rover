import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arjun/ldlidar_ros2_ws/install/a_star_new'
