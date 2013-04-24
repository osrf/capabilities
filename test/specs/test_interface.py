from __future__ import print_function

import os

from capabilities.specs import interface

test_data_dir = os.path.join(os.path.dirname(__file__), 'interfaces')


def check_interface(ci):
    assert ci.spec_version == 1


def check_navigation(ci):
    assert 'Navigation' == ci.name, "Navigation != {0}".format(ci.name)
    assert 'ability to navigate' in ci.description
    check_interface(ci)


def check_rgbcamera(ci):
    assert 'RGBCamera' == ci.name, "RGBCamera != {0}".format(ci.name)
    assert 'exposed by a generic RGBCamera' in ci.description
    check_interface(ci)


def check_minimal(ci):
    assert 'Minimal' == ci.name
    assert 'No description given.' in ci.description
    check_interface(ci)


test_files_map = {
    'Navigation.yaml': check_navigation,
    'RGBCamera.yaml': check_rgbcamera,
    'Minimal.yaml': check_minimal
}


def test_capability_interface_from_file_path():
    for test_file in test_files_map.keys():
        print('running test on file ' + test_file)
        test_file_path = os.path.join(test_data_dir, test_file)
        ci = interface.capability_interface_from_file_path(test_file_path)
        checker = test_files_map[test_file]
        checker(ci)
