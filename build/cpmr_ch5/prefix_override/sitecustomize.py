import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/common/CPMR3/ros2_ws/src/cpmr_ch5/install/cpmr_ch5'
