#!/usr/bin/env python

from __future__ import print_function

import unittest

import rostest

import rospy

from rosservice import get_service_class_by_name

TEST_NAME = 'test_white_black_list'


def call_service(service_name, *args, **kwargs):
    rospy.wait_for_service(service_name)
    service_type = get_service_class_by_name(service_name)
    proxy = rospy.ServiceProxy(service_name, service_type)
    return proxy(*args, **kwargs)


class Test(unittest.TestCase):
    def test_introspection_services(self):
        # get interafaces
        resp = call_service('/capability_server/get_interfaces')
        assert 'minimal_pkg/Minimal' not in resp.interfaces, resp
        assert 'navigation_capability/Navigation' in resp.interfaces, resp
        assert 'differential_mobile_base_capability/DifferentialMobileBase' not in resp.interfaces, resp

if __name__ == '__main__':
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
