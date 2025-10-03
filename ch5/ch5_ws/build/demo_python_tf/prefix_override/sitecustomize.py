import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nayr/FishRos/ch5/ch5_ws/install/demo_python_tf'
