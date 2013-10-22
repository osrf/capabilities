from __future__ import print_function

import os

from ..common import assert_raises_regex

from capabilities.specs import semantic_interface as si

test_data_dir = os.path.join(os.path.dirname(__file__), 'semantic_interfaces')


def check_interface(sci):
    assert sci.spec_version == 1


def check_front(sci):
    assert 'FrontRGBCamera' == sci.name, "FrontRGBCamera != {0}".format(sci.name)
    assert 'This is semantically the Front RGB camera.' in sci.description
    assert 'a_package/RGBCamera' == sci.redefines, sci.redefines
    assert 'front_camera' == sci.global_namespace
    str(sci.remappings)
    check_interface(sci)


def check_minimal(sci):
    assert 'Minimal' == sci.name
    assert 'No description given.' in sci.description
    check_interface(sci)
    str(sci)

test_files_map = {
    # 'file': [extra checker function, expected error, expected error regex]
    'DoesNotExist.yaml': [None, IOError, 'No such file'],
    'InvalidSpecType.yaml': [None, si.InvalidSemanticInterface, 'Invalid spec type'],
    'Minimal.yaml': [check_minimal, None, None],
    'FrontRGBCamera.yaml': [check_front, None, None],
    'NoName.yaml': [None, si.InvalidSemanticInterface, 'No name specified'],
    'NoSpecVersion.yaml': [None, si.InvalidSemanticInterface, 'No spec version specified'],
    'NoSpecType.yaml': [None, si.InvalidSemanticInterface, 'No spec type specified'],
    'NoRedefines.yaml': [None, si.InvalidSemanticInterface, 'No redefined capability specified'],
    'Version2Spec.yaml': [None, si.InvalidSemanticInterface, 'Invalid spec version'],
    'InvalidRemappingType.yaml': [None, si.InvalidSemanticInterface, 'Invalid remapping type'],
    'DictRedefines.yaml': [None, si.InvalidSemanticInterface, 'Invalid redefines, must be a string'],
    'InvalidRedefinesName.yaml': [None, si.InvalidSemanticInterface, 'Invalid spec name for redefines'],
}


def test_semantic_capability_interface_from_file_path():
    default_checker = lambda x: None
    print()  # Keeps the output clean when doing nosetests -s
    for test_file, (checker, expected_exception, expected_exception_regex) in test_files_map.iteritems():
        checker = checker or default_checker
        print('running test on file ' + test_file)
        test_file_path = os.path.join(test_data_dir, test_file)
        with assert_raises_regex(expected_exception, expected_exception_regex):
            sci = si.semantic_capability_interface_from_file_path(test_file_path)
            checker(sci)


def test_semantic_capability_interface_from_file():
    test_file = 'Minimal.yaml'
    checker, expected_exception, expected_exception_regex = test_files_map[test_file]
    test_file_path = os.path.join(test_data_dir, test_file)
    with open(test_file_path, 'r') as f:
        with assert_raises_regex(expected_exception, expected_exception_regex):
            cp = si.semantic_capability_interface_from_file(f)
            checker(cp)


def test_semantic_capability_interface_from_string():
    test_file = 'Minimal.yaml'
    checker, expected_exception, expected_exception_regex = test_files_map[test_file]
    test_file_path = os.path.join(test_data_dir, test_file)
    with open(test_file_path, 'r') as f:
        with assert_raises_regex(expected_exception, expected_exception_regex):
            cp = si.semantic_capability_interface_from_string(f.read())
            checker(cp)
