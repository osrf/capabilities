#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import time
import unittest

from mock import Mock

import rostest

from capabilities import launch_manager

TEST_NAME = 'test_launch_manager'
THIS_DIR = os.path.dirname(__file__)
TEST_DIR = os.path.abspath(os.path.join(THIS_DIR, '..', '..'))

sys.path.insert(0, TEST_DIR)
from unit.common import assert_raises_regex, redirected_stdio

from capabilities.discovery import package_index_from_package_path
from capabilities.discovery import spec_file_index_from_package_index
from capabilities.discovery import spec_index_from_spec_file_index


class Test(unittest.TestCase):
    def test_launch_manager_quiet(self):
        with redirected_stdio():
            workspaces = [os.path.join(TEST_DIR, 'unit', 'discovery_workspaces', 'minimal')]
            package_index = package_index_from_package_path(workspaces)
            spec_file_index = spec_file_index_from_package_index(package_index)
            spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
            assert not errors, errors
            provider = 'minimal_pkg/minimal'
            assert provider in spec_index.providers
            lm = launch_manager.LaunchManager()
            lm._LaunchManager__quiet = True
            lm.run_capability_provider(spec_index.providers[provider], spec_index.provider_paths[provider])
            with assert_raises_regex(RuntimeError, 'is already running'):
                lm.run_capability_provider(spec_index.providers[provider], spec_index.provider_paths[provider])
            with assert_raises_regex(RuntimeError, 'No running launch file with PID of'):
                lm._LaunchManager__stop_by_pid(-1)
            time.sleep(1)  # Allow time for output to be produced
            lm.stop()

    def test_launch_manager_screen(self):
        with redirected_stdio():
            workspaces = [os.path.join(TEST_DIR, 'unit', 'discovery_workspaces', 'minimal')]
            package_index = package_index_from_package_path(workspaces)
            spec_file_index = spec_file_index_from_package_index(package_index)
            spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
            assert not errors, errors
            provider = 'minimal_pkg/minimal'
            assert provider in spec_index.providers
            lm = launch_manager.LaunchManager()
            lm._LaunchManager__screen = True
            lm.run_capability_provider(spec_index.providers[provider], spec_index.provider_paths[provider])
            time.sleep(1)  # Allow time for output to be produced
            lm.stop()

    def test_launch_manager_no_launch_file_provider(self):
        with redirected_stdio():
            workspaces = [os.path.join(THIS_DIR, 'no_launch_file')]
            package_index = package_index_from_package_path(workspaces)
            spec_file_index = spec_file_index_from_package_index(package_index)
            spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
            assert not errors, errors
            provider = 'minimal_pkg/minimal'
            assert provider in spec_index.providers
            lm = launch_manager.LaunchManager()
            lm._LaunchManager__quiet = True
            lm.run_capability_provider(spec_index.providers[provider], spec_index.provider_paths[provider])
            time.sleep(1)  # Allow time for output to be produced
            lm.stop()

    def test_process_monitoring(self):
        lm = launch_manager.LaunchManager()
        try:
            with assert_raises_regex(RuntimeError, 'Unknown process id'):
                proc = Mock()
                proc.pid = -1
                lm._LaunchManager__monitor_process(proc)
        finally:
            lm.stop()

if __name__ == '__main__':
    import rospy
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.unitrun('capabilities', TEST_NAME, Test)
