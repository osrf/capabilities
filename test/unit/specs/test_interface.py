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
    assert sorted(['obstacles', 'inflated_obstacles', 'goal']) == sorted(ci.topics)
    assert ['max_speed'] == ci.dynamic_parameters
    with assert_raises_regex(AttributeError, "can't set attribute"):
        ci.dynamic_parameters = ['max_speed2']
    assert {} == ci.actions
    assert 'get_map' in ci.services.keys()
    assert 'nav_msgs/GetMap' == ci.services['get_map'].type
    ci.provided_topics
    ci.required_topics
    ci.provided_services
    ci.required_services
    ci.provided_actions
    ci.required_actions
    ci.provided_parameters
    ci.required_parameters
    check_interface(ci)
    str(ci.topics.values()[0])


def check_all_interfaces(ci):
    str(ci)


def check_rgbcamera(ci):
    assert 'RGBCamera' == ci.name, "RGBCamera != {0}".format(ci.name)
    assert 'exposed by a generic RGBCamera' in ci.description
    assert {} == ci.required_topics
    check_interface(ci)


def check_minimal(ci):
    assert 'Minimal' == ci.name, ci.name
    assert 'No description given.' in ci.description
    check_interface(ci)
    str(ci)

test_files_map = {
    # 'file': [extra checker function, expected error, expected error regex]
    'AllInterfaces.yaml': [check_all_interfaces, None, None],
    'DoesNotExists.yaml': [None, IOError, 'No such file'],
    'DuplicateAction.yaml': [None, interface.InvalidInterface, 'Interface has action listed twice'],
    'DuplicateDynParameter.yaml': [None, interface.InvalidInterface, 'Interface has dynamic parameter listed twice'],
    'DuplicateParameter.yaml': [None, interface.InvalidInterface, 'Interface has parameter listed twice'],
    'DuplicateService.yaml': [None, interface.InvalidInterface, 'Interface has service listed twice'],
    'DuplicateTopic.yaml': [None, interface.InvalidInterface, 'Interface has topic listed twice'],
    'InvalidDynamicParameter.yaml': [None, interface.InvalidInterface, 'Invalid dynamic_parameters entry'],
    'InvalidInterfaceSection.yaml': [None, interface.InvalidInterface, 'Invalid interface section'],
    'InvalidInterfaceSectionName.yaml': [None, interface.InvalidInterface, "did you mean: 'parameters', or"],
    'InvalidInterfaceSectionName2.yaml': [None, interface.InvalidInterface, "did you mean: 'actions'?"],
    'InvalidInterfaceSectionType.yaml': [None, interface.InvalidInterface, 'section, expected dict got:'],
    'InvalidSpecType.yaml': [None, interface.InvalidInterface, "expected 'interface' got:"],
    'Minimal.yaml': [check_minimal, None, None],
    'Navigation.yaml': [check_navigation, None, None],
    'NoName.yaml': [None, interface.InvalidInterface, 'No name specified'],
    'NoSpecType.yaml': [None, interface.InvalidInterface, 'No spec type specified'],
    'NoSpecVersion.yaml': [None, interface.InvalidInterface, 'No spec version specified'],
    'NoType.yaml': [None, interface.InvalidInterface, 'Topic has no type'],
    'RGBCamera.yaml': [check_rgbcamera, None, None],
    'Version2Spec.yaml': [None, interface.InvalidInterface, 'Invalid spec version'],
}


def test_capability_interface_from_file_path():
    default_checker = lambda x: None
    for test_file, (checker, expected_exception, expected_exception_regex) in test_files_map.iteritems():
        checker = checker or default_checker
        print('running test on file ' + test_file)
        test_file_path = os.path.join(test_data_dir, test_file)
        with assert_raises_regex(expected_exception, expected_exception_regex):
            ci = interface.capability_interface_from_file_path(test_file_path)
            checker(ci)


def test_capability_interface_from_file():
    test_file = 'Minimal.yaml'
    checker, expected_exception, expected_exception_regex = test_files_map[test_file]
    test_file_path = os.path.join(test_data_dir, test_file)
    with open(test_file_path, 'r') as f:
        with assert_raises_regex(expected_exception, expected_exception_regex):
            cp = interface.capability_interface_from_file(f)
            checker(cp)


def test_capability_interface_from_string():
    test_file = 'Minimal.yaml'
    checker, expected_exception, expected_exception_regex = test_files_map[test_file]
    test_file_path = os.path.join(test_data_dir, test_file)
    with open(test_file_path, 'r') as f:
        with assert_raises_regex(expected_exception, expected_exception_regex):
            cp = interface.capability_interface_from_string(f.read())
            checker(cp)


def test_Interface__check_element():
    i = interface.Interface()
    with assert_raises_regex(TypeError, 'Invalid element'):
        i._Interface__check_element(None)


def test_Interface__check_name():
    i = interface.Interface()
    with assert_raises_regex(TypeError, 'Invalid element name'):
        i._Interface__check_name(None)


def test_Interface__add_element():
    i = interface.Interface()
    with assert_raises_regex(ValueError, 'Invalid element kind'):
        i._Interface__add_element('bad_kind', None, None, None)
    with assert_raises_regex(ValueError, 'Invalid grouping'):
        i._Interface__add_element('topic', 'foo', interface.InterfaceElement('foo', 'topic'), 'invalid_group')


def test_InterfaceElement():
    with assert_raises_regex(ValueError, 'Invalid element kind'):
        interface.InterfaceElement('foo', 'invalid_type')
