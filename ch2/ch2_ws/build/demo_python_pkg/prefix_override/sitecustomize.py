import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nayr/ch2/ch2_ws/install/demo_python_pkg'
