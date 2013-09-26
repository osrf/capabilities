#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import unittest

import rostest

import rospy
from rospy.service import ServiceException

from rosservice import get_service_class_by_name

TEST_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

sys.path.insert(0, TEST_DIR)
from unit.common import assert_raises

TEST_NAME = 'test_ros_services'

from capabilities.service_discovery import spec_index_from_service
from capabilities.msg import CapabilityEvent


def call_service(service_name, *args, **kwargs):
    rospy.wait_for_service(service_name)
    service_type = get_service_class_by_name(service_name)
    proxy = rospy.ServiceProxy(service_name, service_type)
    return proxy(*args, **kwargs)


class Test(unittest.TestCase):
    def test_introspection_services(self):
        # Reload
        call_service('/capability_server/reload_capabilities')
        # get interafaces
        resp = call_service('/capability_server/get_interfaces')
        assert 'minimal_pkg/Minimal' in resp.interfaces, resp
        # get providers by interface
        resp = call_service('/capability_server/get_providers', 'minimal_pkg/Minimal')
        assert 'minimal_pkg/minimal' in resp.providers, resp
        # get all providers
        resp = call_service('/capability_server/get_providers', '')
        assert 'minimal_pkg/minimal' in resp.providers, resp
        # get semantic interfaces
        resp = call_service('/capability_server/get_semantic_interfaces')
        assert 'minimal_pkg/SpecificMinimal' in resp.semantic_interfaces, resp
        # get semantic interfaces by interface
        resp = call_service('/capability_server/get_semantic_interfaces', 'minimal_pkg/Minimal')
        assert 'minimal_pkg/SpecificMinimal' in resp.semantic_interfaces, resp

    def test_start_stop_capabilities(self):
        # fail to start interface without a provider
        with assert_raises(ServiceException):
            call_service('/capability_server/start_capability', 'no_provider_pkg/Minimal2', '')
        # fail to start interface which doesn't exist
        with assert_raises(ServiceException):
            call_service('/capability_server/start_capability', 'minimal_pkg/Minimal2', 'minimal_pkg/minimal')
        # start an interface by explicitly giving a provider
        call_service('/capability_server/start_capability', 'minimal_pkg/Minimal', 'minimal_pkg/minimal')
        # Wait a reasonable amount of time for roslaunch to startup
        # If it doesn't fully startup you get a traceback, but it will not affect the test
        rospy.sleep(2)
        # get list of running capabilities
        resp = call_service('/capability_server/get_running_capabilities')
        assert len(resp.running_capabilities) > 0, resp
        capability = resp.running_capabilities[0].capability
        assert 'minimal_pkg/Minimal' == capability.capability, capability.capability
        # stop the capability started above
        call_service('/capability_server/stop_capability', 'minimal_pkg/Minimal')
        # fail to stop a capability which isn't running
        with assert_raises(ServiceException):
            call_service('/capability_server/stop_capability', 'minimal_pkg/Minimal')
        # fail to start a capability with a provider which doesn't exist
        with assert_raises(ServiceException):
            call_service('/capability_server/start_capability', 'minimal_pkg/Minimal', 'minimal_pkg/minimal2')
        # start a capability without specifying a provider
        call_service('/capability_server/start_capability', 'minimal_pkg/Minimal', '')
        call_service('/capability_server/stop_capability', 'minimal_pkg/Minimal')

    def test_service_discovery(self):
        # get spec index via a service call (This tests the handling in the server)
        si, errors = spec_index_from_service()
        assert not errors
        assert 'minimal_pkg/Minimal' in si.interfaces

    def test_external_event(self):
        # publish even from external rospy instance
        pub = rospy.Publisher("~events", CapabilityEvent)
        rospy.sleep(1)
        pub.publish(CapabilityEvent())
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
