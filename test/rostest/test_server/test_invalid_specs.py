#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import unittest

import rospy
import rostest

from capabilities import server

TEST_NAME = 'test_invalid_specs'
TEST_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

sys.path.insert(0, TEST_DIR)
from unit.common import assert_raises

from capabilities.msg import CapabilityEvent


class Test(unittest.TestCase):
    def test_invalid_specs(self):
        invalid_specs_dir = os.path.join(TEST_DIR, 'unit', 'discovery_workspaces', 'invalid_specs')
        ros_package_path = [invalid_specs_dir]
        capability_server = server.CapabilityServer(ros_package_path)
        capability_server._CapabilityServer__load_capabilities()
        capability_server._CapabilityServer__populate_default_providers()
        capability_server._CapabilityServer__stop_capability('not_a_running_capability')
        capability_server.shutdown()

    def test_no_default_provider_pedantic(self):
        no_default_provider = os.path.join(TEST_DIR, 'rostest', 'test_server', 'no_default_provider')
        ros_package_path = [no_default_provider]
        rospy.set_param('~missing_default_provider_is_an_error', True)
        capability_server = server.CapabilityServer(ros_package_path)
        capability_server._CapabilityServer__load_capabilities()
        with assert_raises(SystemExit):
            capability_server._CapabilityServer__populate_default_providers()
        capability_server.shutdown()

    def test_no_default_provider(self):
        no_default_provider = os.path.join(TEST_DIR, 'rostest', 'test_server', 'no_default_provider')
        ros_package_path = [no_default_provider]
        rospy.set_param('~missing_default_provider_is_an_error', False)
        capability_server = server.CapabilityServer(ros_package_path)
        capability_server._CapabilityServer__load_capabilities()
        capability_server._CapabilityServer__populate_default_providers()
        capability_server.shutdown()

    def test_invalid_default_provider(self):
        minimal_dir = os.path.join(TEST_DIR, 'unit', 'discovery_workspaces', 'minimal')
        ros_package_path = [minimal_dir]
        rospy.set_param('~defaults/minimal_pkg/Minimal', 'minimal_pkg/not_a_valid_provider')
        capability_server = server.CapabilityServer(ros_package_path)
        capability_server._CapabilityServer__load_capabilities()
        with assert_raises(SystemExit):
            capability_server._CapabilityServer__populate_default_providers()
        capability_server.shutdown()

    def test_wrong_default_provider(self):
        dc_dir = os.path.join(TEST_DIR, 'unit', 'discovery_workspaces', 'dependent_capabilities')
        ros_package_path = [dc_dir]
        rospy.set_param('~defaults/navigation_capability/Navigation',
                        'differential_mobile_base_capability/faux_differential_mobile_base')
        capability_server = server.CapabilityServer(ros_package_path)
        capability_server._CapabilityServer__load_capabilities()
        with assert_raises(SystemExit):
            capability_server._CapabilityServer__populate_default_providers()
        capability_server.shutdown()

    def test_event_handler(self):
        invalid_specs_dir = os.path.join(TEST_DIR, 'unit', 'discovery_workspaces', 'invalid_specs')
        ros_package_path = [invalid_specs_dir]
        capability_server = server.CapabilityServer(ros_package_path)
        capability_server._CapabilityServer__load_capabilities()
        capability_server._CapabilityServer__populate_default_providers()
        rospy.Subscriber('~events', CapabilityEvent, capability_server.handle_capability_events)
        pub = rospy.Publisher("~events", CapabilityEvent, queue_size=1000)
        rospy.sleep(1)
        msg = CapabilityEvent()
        msg.capability = 'some_pkg/NotACapability'
        msg.provider = 'doesnt matter'
        msg.type = 'doesnt matter'
        pub.publish(msg)
        rospy.sleep(1)  # Allow time for the publish to happen
        capability_server.shutdown()

if __name__ == '__main__':
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
