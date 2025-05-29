import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roboticist/0_Projects/tail_mate/install/sjtu_drone_control'
