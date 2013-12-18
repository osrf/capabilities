#!/usr/bin/env python

from __future__ import print_function

import unittest

import rostest

import rospy

from test_ros_services import assert_raises
from test_ros_services import call_service
from test_ros_services import ServiceException

TEST_NAME = 'test_dependent_capabilities'


class Test(unittest.TestCase):
    def test_start_stop_dependent_capabilities(self):
        call_service('/capability_server/start_capability',
                     'navigation_capability/Navigation',
                     'navigation_capability/faux_navigation')
        rospy.sleep(6)  # Wait for the system to settle
        resp = call_service('/capability_server/get_running_capabilities')
        result = [x.capability.capability for x in resp.running_capabilities]
        expected = ['navigation_capability/Navigation',
                    'differential_mobile_base_capability/DifferentialMobileBase']
        assert sorted(result) == sorted(expected), sorted(result)
        call_service('/capability_server/start_capability',
                     'minimal_pkg/Minimal',
                     'minimal_pkg/minimal')
        rospy.sleep(1)
        call_service('/capability_server/stop_capability',
                     'minimal_pkg/Minimal')
        rospy.sleep(2)
        call_service('/capability_server/stop_capability', 'navigation_capability/Navigation')
        rospy.sleep(1)  # Wait for the system to shutdown, so coverage can work
        # fail to call provider with invalid dependent provider
        with assert_raises(ServiceException):
            call_service('/capability_server/start_capability',
                         'navigation_capability/Navigation',
                         'navigation_capability/faux_navigation_invalid_preferred_provider')

    def test_stop_base_capability(self):
        '''
        Stopping a base capability should stop all dependent capabilities too.
        '''
        call_service('/capability_server/start_capability',
                     'navigation_capability/Navigation',
                     'navigation_capability/faux_navigation')
        rospy.sleep(6)  # Wait for the system to settle
        resp = call_service('/capability_server/get_running_capabilities')
        result = [x.capability.capability for x in resp.running_capabilities]
        expected = ['navigation_capability/Navigation',
                    'differential_mobile_base_capability/DifferentialMobileBase']
        self.assertEqual(sorted(expected), sorted(expected))
        call_service('/capability_server/stop_capability',
                     'differential_mobile_base_capability/DifferentialMobileBase')
        resp = call_service('/capability_server/get_running_capabilities')
        # The list of running capabilities should be empty
        self.assertEqual([], resp.running_capabilities)
 

if __name__ == '__main__':
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
