#!/usr/bin/env python

from __future__ import print_function

import unittest

import rostest

import rospy

from test_ros_services import call_service
from test_ros_services import wait_for_capability_server

TEST_NAME = 'test_default_provider'


class Test(unittest.TestCase):
    def test_default_provider(self):
        assert wait_for_capability_server(10)
        call_service('/capability_server/start_capability', 'no_default_provider_pkg/Minimal', '')
        rospy.sleep(1)  # Wait for the system to settle
        resp = call_service('/capability_server/get_running_capabilities')
        result = [x.capability.provider for x in resp.running_capabilities]
        retry_count = 0
        while not result:
            retry_count += 1
            if retry_count == 10:
                break
            # Retry, sometimes the system is really slow...
            rospy.sleep(1)
            resp = call_service('/capability_server/get_running_capabilities')
            result = [x.capability.provider for x in resp.running_capabilities]
        expected = ['no_default_provider_pkg/minimal']
        assert sorted(result) == sorted(expected), (sorted(result), sorted(expected))

if __name__ == '__main__':
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
