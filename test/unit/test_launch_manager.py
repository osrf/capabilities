from __future__ import print_function

import os
import sys

try:
    from .common import environment

    from capabilities import launch_manager

    TEST_DIR = os.path.dirname(__file__)

    def test_which():
        fake_bin = os.path.join(TEST_DIR, 'fake_root', 'bin')
        with environment({'PATH': fake_bin}):
            ls_path = os.path.join(fake_bin, 'ls')
            assert ls_path == launch_manager.which('ls')
            assert ls_path == launch_manager.which(ls_path)
            assert None == launch_manager.which('bad')

except ImportError as exc:
    if 'rospy' not in str(exc) and 'No module named srv' not in str(exc):
        raise
    print("Skipping test_launch_manager.py because ROS depenencies not imported: " + str(exc), file=sys.stderr)
