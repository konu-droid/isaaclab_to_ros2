import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/konu/Documents/isaaclab_to_ros2/ros2_convertor_ws/install/isaaclab_to_ros2'
