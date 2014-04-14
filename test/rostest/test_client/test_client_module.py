#!/usr/bin/env python

from __future__ import print_function

# import os
# import sys
import unittest

import rostest

from capabilities.client import CapabilitiesClient

# TEST_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# sys.path.insert(0, TEST_DIR)
# from unit.common import assert_raises

TEST_NAME = 'test_client_module'


class Test(unittest.TestCase):
    def test_client_module(self):
        c = CapabilitiesClient()
        c.wait_for_services(3)
        c.use_capability('minimal_pkg/Minimal', 'minimal_pkg/minimal')
        c.use_capability('minimal_pkg/Minimal', 'minimal_pkg/minimal')
        c.free_capability('minimal_pkg/Minimal')
        c.free_capability('not_a_pkg/NotACap')
        c.shutdown()

if __name__ == '__main__':
    import rospy
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
