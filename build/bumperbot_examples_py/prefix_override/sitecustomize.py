import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sushant-ros/my_bots/src/install/bumperbot_examples_py'
