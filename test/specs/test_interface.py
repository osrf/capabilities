from __future__ import print_function

import os

from ..common import assert_raises_regex

from capabilities.specs import interface

test_data_dir = os.path.join(os.path.dirname(__file__), 'interfaces')


def check_interface(ci):
    assert ci.spec_version == 1


def check_navigation(ci):
    assert 'Navigation' == ci.name, "Navigation != {0}".format(ci.name)
    assert 'ability to navigate' in ci.description
    assert sorted(['obstacles', 'inflated_obstacles', 'goal']) == sorted([t.name for t in ci.topics])
    assert ['max_speed'] == ci.dynamic_parameters
    with assert_raises_regex(AttributeError, "can't set attribute"):
        ci.dynamic_parameters = ['max_speed2']
    assert {} == ci.actions
    assert 'get_map' in ci.services.keys()
    assert 'nav_msgs/GetMap' == ci.services['get_map'].type
    check_interface(ci)


def check_rgbcamera(ci):
    assert 'RGBCamera' == ci.name, "RGBCamera != {0}".format(ci.name)
    assert 'exposed by a generic RGBCamera' in ci.description
    assert {} == ci.required_topics
    check_interface(ci)


def check_minimal(ci):
    assert 'Minimal' == ci.name
    assert 'No description given.' in ci.description
    check_interface(ci)

test_files_map = {
    # 'file': [extra checker function, expected error, expected error regex]
    'DoesNotExists.yaml': [None, IOError, 'No such file'],
    'DuplicateAction.yaml': [None, interface.InvalidInterface, 'Interface has action listed twice'],
    'DuplicateDynParameter.yaml': [None, interface.InvalidInterface, 'Interface has dynamic parameter listed twice'],
    'DuplicateParameter.yaml': [None, interface.InvalidInterface, 'Interface has parameter listed twice'],
    'DuplicateService.yaml': [None, interface.InvalidInterface, 'Interface has service listed twice'],
    'DuplicateTopic.yaml': [None, interface.InvalidInterface, 'Interface has topic listed twice'],
    'InvalidDynamicParameter.yaml': [None, interface.InvalidInterface, 'Invalid dynamic_parameters entry'],
    'InvalidInterfaceSection.yaml': [None, interface.InvalidInterface, 'Invalid interface section'],
    'InvalidInterfaceSectionName.yaml': [None, interface.InvalidInterface, "did you mean: 'dynamic_parameters',"],
    'InvalidInterfaceSectionType.yaml': [None, interface.InvalidInterface, 'section, expected dict got:'],
    'Minimal.yaml': [check_minimal, None, None],
    'Navigation.yaml': [check_navigation, None, None],
    'NoName.yaml': [None, interface.InvalidInterface, 'No name specified'],
    'NoSpecVersion.yaml': [None, interface.InvalidInterface, 'No spec version specified'],
    'NoType.yaml': [None, interface.InvalidInterface, 'Topic has no type'],
    'RGBCamera.yaml': [check_rgbcamera, None, None],
    'Version2Spec.yaml': [None, interface.InvalidInterface, 'Invalid spec version'],
}


def test_capability_interface_from_file_path():
    default_checker = lambda x: None
    for test_file in test_files_map.keys():
        checker = test_files_map[test_file][0] or default_checker
        expected_exception = test_files_map[test_file][1]
        expected_exception_regex = test_files_map[test_file][2]
        print('running test on file ' + test_file)
        test_file_path = os.path.join(test_data_dir, test_file)
        with assert_raises_regex(expected_exception, expected_exception_regex):
            ci = interface.capability_interface_from_file_path(test_file_path)
            checker(ci)
