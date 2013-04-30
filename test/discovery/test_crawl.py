from __future__ import print_function

import os

from ..common import assert_raises_regex

from capabilities.specs import interface
from capabilities.discovery import _build_package_dict, _build_file_index, list_interfaces, list_interface_files


def DISABLEDtest_list_interfaces():
    list_interfaces()



def test_build_index():
    ros_package_path = ['test/discovery/crawl']
    pkgs = _build_package_dict(ros_package_path)
    result = _build_file_index(pkgs)
    #assert 'interface_pkg' in result['interfaces']
    print("result", result)


def test_list_interfaces():
    ros_package_path = ['test/discovery/crawl']
    pkgs = _build_package_dict(ros_package_path)
    file_index = _build_file_index(pkgs)
    interfaces = list_interfaces(file_index)
    #assert 'interface_pkg' in interfaces, "No interface_pkg found"


def test_list_interface_files():
    ros_package_path = ['test/discovery/crawl']
    pkgs = _build_package_dict(ros_package_path)
    file_index = _build_file_index(pkgs)
    interface_files = list_interface_files(file_index)
    print(interface_files)
