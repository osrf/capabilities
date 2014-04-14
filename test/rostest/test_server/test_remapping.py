#!/usr/bin/env python

from __future__ import print_function

import unittest

import rostest

import rospy
from rospy.service import ServiceException

from test_ros_services import assert_raises
from test_ros_services import call_service

TEST_NAME = 'test_remapping'


class Test(unittest.TestCase):
    def test_remapping(self):
        with assert_raises(ServiceException):
            call_service('/capability_server/get_remappings', 'robot_base_pkg/FrontDistance')
        call_service('/capability_server/start_capability', 'robot_base_pkg/FrontDistance', '')
        resp = call_service('/capability_server/get_remappings', 'robot_base_pkg/FrontDistance')
        assert len(resp.topics) == 1, resp
        assert ('front/distance', 'robot/front/distance') in [(t.key, t.value) for t in resp.topics], resp
        resp = call_service('/capability_server/get_remappings', 'robot_base_pkg/front_distance')
        assert resp.topics, resp
        assert ('front/distance', 'robot/front/distance') in [(t.key, t.value) for t in resp.topics], resp
        resp = call_service('/capability_server/get_remappings', 'robot_base_pkg/Distance')
        assert resp.topics, resp
        assert ('distance', 'robot/front/distance') in [(t.key, t.value) for t in resp.topics], resp

if __name__ == '__main__':
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
