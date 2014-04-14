#!/usr/bin/env python

from __future__ import print_function

import unittest

import rostest

import rospy
from rospy.service import ServiceException

from test_ros_services import assert_raises
from test_ros_services import call_service
from test_ros_services import wait_for_capability_server

from capabilities.client import CapabilitiesClient

TEST_NAME = 'test_remapping'


def wait_for_result_to_happen(expected, initial_result, tries=10, sleep_period=1):
    result = initial_result
    count = 0
    while count != tries and sorted(result) != sorted(expected):
        rospy.sleep(sleep_period)
        count += 1
        resp = call_service('/capability_server/get_running_capabilities')
        result = [x.capability.capability for x in resp.running_capabilities]
    return result


class Test(unittest.TestCase):
    def test_use_and_free_capability(self):
        wait_for_capability_server(3)
        c = CapabilitiesClient()
        c.wait_for_services(timeout=3.0)
        # Give invalid bond id to use_capability
        with assert_raises(ServiceException):
            call_service('/capability_server/use_capability', 'minimal_pkg/Minimal', '', 'invalid_bond_id')
        # Try to use a non-existent cap
        with assert_raises(ServiceException):
            c.use_capability('not_a_pkg/NotACap')
        # Use cap and wait for it to be running
        c.use_capability('minimal_pkg/Minimal', 'minimal_pkg/minimal')
        expected = ['minimal_pkg/Minimal']
        result = wait_for_result_to_happen(expected, [])
        assert sorted(result) == sorted(expected), (sorted(result), sorted(expected))
        # Try to use it with a different provider
        with assert_raises(ServiceException):
            c.use_capability('minimal_pkg/Minimal', 'minimal_pkg/specific_minimal')
        # Use it a second time, free it once, assert it is stil there
        c.use_capability('minimal_pkg/Minimal')
        c.free_capability('minimal_pkg/Minimal')
        expected = []
        result = wait_for_result_to_happen(expected, ['minimal_pkg/Minimal'], tries=5)
        assert sorted(result) != sorted(expected), (sorted(result), sorted(expected))
        # Directly call ~free_capability with an invalid bond_id
        with assert_raises(ServiceException):
            call_service('/capability_server/free_capability', 'minimal_pkg/Minimal', 'invalid_bond_id')
        # Free it again and assert it goes down
        c.free_capability('minimal_pkg/Minimal')
        expected = []
        result = wait_for_result_to_happen(expected, ['minimal_pkg/Minimal'])
        assert sorted(result) == sorted(expected), (sorted(result), sorted(expected))
        # Try to over free it and get an exception
        with assert_raises(ServiceException):
            c.free_capability('minimal_pkg/Minimal')
        # Use it again, then break the bond and assert it goes down because of that
        c.use_capability('minimal_pkg/Minimal')
        expected = ['minimal_pkg/Minimal']
        result = wait_for_result_to_happen(expected, [])
        assert sorted(result) == sorted(expected), (sorted(result), sorted(expected))
        c._bond.break_bond()
        expected = []
        result = wait_for_result_to_happen(expected, ['minimal_pkg/Minimal'])
        assert sorted(result) == sorted(expected), (sorted(result), sorted(expected))

if __name__ == '__main__':
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
