from __future__ import print_function

import os

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
    assert not errors, errors
    expected = sorted([
        'minimal_pkg/Minimal',
        'minimal_pkg/minimal',
        'minimal_pkg/SpecificMinimal',
        'minimal_pkg/specific_minimal'
    ])
    assert sorted(spec_index.specs.keys()) == expected
    assert sorted(spec_index.provider_names) == ['minimal_pkg/minimal', 'minimal_pkg/specific_minimal']
    assert 'minimal_pkg/Minimal' in spec_index.interface_paths
    assert 'minimal_pkg/minimal' in spec_index.provider_paths
    assert 'minimal_pkg/SpecificMinimal' in spec_index.semantic_interface_paths


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
