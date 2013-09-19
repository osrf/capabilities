#!/usr/bin/env python

from __future__ import print_function

import unittest

import rostest

from capabilities.service_discovery import spec_index_from_service

TEST_NAME = 'test_spec_index_from_service'


class Test(unittest.TestCase):
    def test_spec_index_from_service(self):
        si, errors = spec_index_from_service()
        assert not errors
        assert 'minimal_pkg/Minimal' in si.interfaces

if __name__ == '__main__':
    import rospy
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
