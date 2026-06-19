import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/raybeak/Desktop/ahShitHereWeGoAgain_ws/src/install/py_pkg'
