from __future__ import print_function

import os

from .common import assert_raises

from capabilities import discovery
from capabilities.discovery import package_index_from_package_path
from capabilities.discovery import spec_file_index_from_package_index
from capabilities.discovery import spec_index_from_spec_file_index

test_data_dir = os.path.join(os.path.dirname(__file__), 'discovery_workspaces')


def test_load_minimal():
    print('Testing minimal workspace')
    workspaces = [os.path.join(test_data_dir, 'minimal')]
    package_index = package_index_from_package_path(workspaces)
    spec_file_index = spec_file_index_from_package_index(package_index)
    spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
    assert not errors
    expected = sorted(['Minimal', 'minimal', 'SpecificMinimal', 'specific_minimal'])
    assert sorted(spec_index.specs.keys()) == expected
    assert spec_index.get_containing_package_name('Minimal') == 'minimal_pkg'
    with assert_raises(RuntimeError):
        spec_index.get_containing_package_name('NotASpecName')
    assert sorted(spec_index.provider_names) == ['minimal', 'specific_minimal']
    assert 'minimal' in spec_index.provider_paths
    assert 'SpecificMinimal' in spec_index.semantic_interface_paths


def test_load_invalid_specs():
    print('Testing invalid_specs workspace')
    workspaces = [os.path.join(test_data_dir, 'invalid_specs')]
    package_index = package_index_from_package_path(workspaces)
    spec_file_index = spec_file_index_from_package_index(package_index)
    spec_index, errors = spec_index_from_spec_file_index(spec_file_index)


def test_load_missing_interface():
    print('Testing missing_interface workspace')
    workspaces = [os.path.join(test_data_dir, 'missing_interface')]
    package_index = package_index_from_package_path(workspaces)
    spec_file_index = spec_file_index_from_package_index(package_index)
    spec_index, errors = spec_index_from_spec_file_index(spec_file_index)


def test_load_duplicate_names():
    print('Testing duplicate_names workspace')
    workspaces = [os.path.join(test_data_dir, 'duplicate_names')]
    package_index = package_index_from_package_path(workspaces)
    spec_file_index = spec_file_index_from_package_index(package_index)
    spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
    error_types = [type(x) for x in errors]
    assert discovery.DuplicateNameException in error_types, error_types


def test_load_duplicate_names_semantic():
    print('Testing duplicate_names_semantic workspace')
    workspaces = [os.path.join(test_data_dir, 'duplicate_names_semantic')]
    package_index = package_index_from_package_path(workspaces)
    spec_file_index = spec_file_index_from_package_index(package_index)
    spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
    error_types = [type(x) for x in errors]
    assert discovery.DuplicateNameException in error_types, error_types
