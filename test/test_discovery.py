from __future__ import print_function

import sys

try:
    import os

    from .common import assert_raises

    from capabilities import discovery
    from capabilities.discovery import package_index_from_package_path
    from capabilities.discovery import spec_file_index_from_package_index
    from capabilities.discovery import spec_index_from_spec_file_index

    test_data_dir = os.path.join(os.path.dirname(__file__), 'discovery_workspaces')

    def test_load_minimal():
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

    def test_load_invalid_specs():
        workspaces = [os.path.join(test_data_dir, 'invalid_specs')]
        package_index = package_index_from_package_path(workspaces)
        spec_file_index = spec_file_index_from_package_index(package_index)
        spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
        assert discovery.InterfaceNameNotFoundException in [type(x) for x in errors]

    def test_load_duplicate_names():
        workspaces = [os.path.join(test_data_dir, 'duplicate_names')]
        package_index = package_index_from_package_path(workspaces)
        spec_file_index = spec_file_index_from_package_index(package_index)
        spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
        error_types = [type(x) for x in errors]
        assert discovery.DuplicateNameException in error_types, error_types

    def test_load_duplicate_names_semantic():
        workspaces = [os.path.join(test_data_dir, 'duplicate_names_semantic')]
        package_index = package_index_from_package_path(workspaces)
        spec_file_index = spec_file_index_from_package_index(package_index)
        spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
        error_types = [type(x) for x in errors]
        assert discovery.DuplicateNameException in error_types, error_types

    def test_load_duplicate_names_multi_package():
        workspaces = [os.path.join(test_data_dir, 'duplicate_names_multi_package')]
        package_index = package_index_from_package_path(workspaces)
        spec_file_index = spec_file_index_from_package_index(package_index)
        spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
        error_types = [type(x) for x in errors]
        assert discovery.DuplicateNameException in error_types, error_types
except ImportError as e:
    if 'rospy' not in str(e) and 'No module named srv' not in str(e):
        raise
    print("Skipping test_discovery.py because ROS depenencies not imported: " + str(e), file=sys.stderr)
