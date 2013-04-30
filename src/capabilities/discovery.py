from __future__ import print_function

import os
from catkin_pkg.packages import find_packages


def _get_ros_package_paths():
    """ Load the ROS_PACKAGE_PATH from the environment """
    return os.getenv('ROS_PACKAGE_PATH', '').split(':')


def _build_package_dict(ros_package_path=_get_ros_package_paths()):
    """
    Find all packages on the ROS_PACKAGE_PATH.
    @return a dict {full_path_to_package: pkg object}
    """
    result = {}
    for path in ros_package_path:
        for pkg_name, pkg in find_packages(path).items():
            result[os.path.abspath(os.path.join(path, pkg_name))] = pkg
    return result


def _build_file_index(pkgs=_build_package_dict()):
    """
    Build a full index of capabiliites.  Returns a dict with three
    elements, 'interfaces', 'providers', and 'semantic_interfaces'

    Each element of the above dict will have:
    {full_path_to_package: [{'file': relative_filename,
                             'package': catkin_pkg pkg object}]
    """
    interfaces = {}
    providers = {}
    semantic_interfaces = {}
    for pkg_name, pkg in pkgs.items():
        for e in pkg.exports:
            t = e.tagname
            a = e.attributes
            c = e.content
            payload = {'file': c,
                       'package': pkg}
            if t == 'capability_interface':
                if pkg_name in interfaces:
                    interfaces[pkg_name].append(payload)
                else:
                    interfaces[pkg_name] = [payload]
            elif t == 'capability_provider':
                if pkg_name in providers:
                    providers[pkg_name].append(payload)
                else:
                    providers[pkg_name] = [payload]
            elif t == 'capability_semantic_interface':
                if pkg_name in semantic_interfaces:
                    semantic_interfaces[pkg_name].append(payload)
                else:
                    semantic_interfaces[pkg_name] = [payload]
    return {'interfaces': interfaces,
            'providers': providers,
            'semantic_interfaces': semantic_interfaces}


def list_interfaces(file_index=_build_file_index()):
    """
    Each element of the returned dict will have:
    {full_path_to_package: [{'file': relative_filename,
                             'package': catkin_pkg pkg object}]
    """
    interfaces = file_index['interfaces']
    return interfaces


def list_interface_files(file_index=_build_file_index()):
    """
    returns a list of all interface filenames
    """
    interfaces = file_index['interfaces']
    result = []
    for p, v in interfaces.items():
        result.extend([os.path.join(p, payload['file'])  for payload in v])
    return result


def list_providers(file_index=_build_file_index()):
    """
    Each element of the returned dict will have:
    {full_path_to_package: [{'file': relative_filename,
                             'package': catkin_pkg pkg object}]
    """
    providers = file_index['providers']
    return providers


def list_provider_files(file_index=_build_file_index()):
    """
    returns a list of all provider filenames
    """
    providers = file_index['providers']
    result = []
    for p, v in providers.items():
        result.extend([os.path.join(p, payload['file'])  for payload in v])
    return result


def list_semantic_interfaces(file_index=_build_file_index()):
    """
    Each element of the returned dict will have:
    {full_path_to_package: [{'file': relative_filename,
                             'package': catkin_pkg pkg object}]
    """
    semantic_interfaces = file_index['semantic_interfaces']
    return semantic_interfaces


def list_semantic_interface_files(file_index=_build_file_index()):
    """
    returns a list of all semantic_interface filenames
    """
    semantic_interfaces = file_index['semantic_interfaces']
    result = []
    for p, v in semantic_interfaces.items():
        result.extend([os.path.join(p, payload['file'])  for payload in v])
    return result
