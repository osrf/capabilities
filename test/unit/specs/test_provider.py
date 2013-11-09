from __future__ import print_function

import os

from ..common import assert_raises_regex

from capabilities.specs import provider

test_data_dir = os.path.join(os.path.dirname(__file__), 'providers')


def check_interface(cp):
    assert cp.spec_version == 1


def check_navigation(cp):
    assert 'navigation_nav_stack' == cp.name, "navigation_nav_stack != {0}".format(cp.name)
    assert 'ability to navigate' in cp.description
    assert cp.implements == 'navigation/Navigation', cp.implements
    assert cp.launch_file == 'launch/navigation_nav_stack.launch'
    assert cp.depends_on('laser_capability/LaserObservation'), cp.dependencies
    str(cp.dependencies['laser_capability/LaserObservation'])
    assert 'scan' in cp.remappings
    assert 'nav_stack/scan' == cp.remappings['scan']
    assert 'hokuyo_capability/hokuyo_base' == cp.dependencies['laser_capability/LaserObservation'].provider
    with assert_raises_regex(AttributeError, "can't set attribute"):
        cp.dependencies = {'SomeInterface': {}}
    check_interface(cp)


def check_minimal(cp):
    assert 'minimal' == cp.name
    assert 'No description given.' in cp.description
    check_interface(cp)
    str(cp)

test_files_map = {
    # 'file': [extra checker function, expected error, expected error regex]
    'does_not_exist.yaml': [None, IOError, 'No such file'],
    'invalid_depends_on_conditional.yaml': [None, provider.InvalidProvider, 'Invalid depends_on interface condition'],
    'invalid_depends_on_conditional_section.yaml': [None, provider.InvalidProvider, 'depends_on conditional section'],
    'invalid_depends_on_section.yaml': [None, provider.InvalidProvider, 'Invalid depends_on section'],
    'invalid_implements_name.yaml': [None, provider.InvalidProvider, 'Invalid spec name for implements'],
    'invalid_nodelet_manager.yaml': [None, provider.InvalidProvider, 'Invalid nodelet_manager'],
    'invalid_remapping.yaml': [None, provider.InvalidProvider, 'Invalid remappings section'],
    'invalid_remapping_duplicate.yaml': [None, provider.InvalidProvider, 'is remapped twice, but to different values'],
    'invalid_spec_type.yaml': [None, provider.InvalidProvider, 'Invalid spec type'],
    'minimal.yaml': [check_minimal, None, None],
    'navigation_nav_stack.yaml': [check_navigation, None, None],
    'no_implements.yaml': [None, provider.InvalidProvider, 'No implements specified'],
    'no_name.yaml': [None, provider.InvalidProvider, 'No name specified'],
    'no_spec_type.yaml': [None, provider.InvalidProvider, 'No spec type specified'],
    'no_spec_version.yaml': [None, provider.InvalidProvider, 'No spec version specified'],
    'version_2_spec.yaml': [None, provider.InvalidProvider, 'Invalid spec version'],
}


def test_capability_provider_from_file_path():
    default_checker = lambda x: None
    print()  # Keeps the output clean when doing nosetests -s
    for test_file, (checker, expected_exception, expected_exception_regex) in test_files_map.iteritems():
        checker = checker or default_checker
        print('running test on file ' + test_file)
        test_file_path = os.path.join(test_data_dir, test_file)
        with assert_raises_regex(expected_exception, expected_exception_regex):
            cp = provider.capability_provider_from_file_path(test_file_path)
            checker(cp)


def test_capability_provider_from_file():
    test_file = 'minimal.yaml'
    checker, expected_exception, expected_exception_regex = test_files_map[test_file]
    test_file_path = os.path.join(test_data_dir, test_file)
    with open(test_file_path, 'r') as f:
        with assert_raises_regex(expected_exception, expected_exception_regex):
            cp = provider.capability_provider_from_file(f)
            checker(cp)


def test_capability_provider_from_string():
    test_file = 'minimal.yaml'
    checker, expected_exception, expected_exception_regex = test_files_map[test_file]
    test_file_path = os.path.join(test_data_dir, test_file)
    with open(test_file_path, 'r') as f:
        with assert_raises_regex(expected_exception, expected_exception_regex):
            cp = provider.capability_provider_from_string(f.read())
            checker(cp)
