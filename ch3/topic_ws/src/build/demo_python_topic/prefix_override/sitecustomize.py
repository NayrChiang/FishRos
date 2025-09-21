import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nayr/FishRos/ch3/topic_ws/src/install/demo_python_topic'
