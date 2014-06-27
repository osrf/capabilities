#!/usr/bin/env python

from __future__ import print_function

import unittest

import rostest

from capabilities.client import CapabilitiesClient
from capabilities.client import CapabilityNotRunningException

TEST_NAME = 'test_client_module'


class Test(unittest.TestCase):
    def test_client_module(self):
        c = CapabilitiesClient()
        c.wait_for_services(3)
        c.use_capability('minimal_pkg/Minimal', 'minimal_pkg/minimal')
        c.use_capability('minimal_pkg/Minimal', 'minimal_pkg/minimal')
        c.free_capability('minimal_pkg/Minimal')
        with self.assertRaises(CapabilityNotRunningException):
            c.free_capability('not_a_pkg/NotACap')
        c.shutdown()

if __name__ == '__main__':
    import rospy
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
