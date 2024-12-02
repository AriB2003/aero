import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zaraius/Documents/test/aero/Electrical 24-25/dev_ws/install/my_package'
