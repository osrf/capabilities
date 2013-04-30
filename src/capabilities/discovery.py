from __future__ import print_function

import os
from catkin_pkg.packages import find_packages


def _get_ros_package_paths():
    return os.getenv('ROS_PACKAGE_PATH', '').split(':')

def _build_package_dict(ros_package_path=_get_ros_package_paths()):
    result = {}
    for path in ros_package_path:
        for pkg_name, pkg in find_packages(path).items():
            result[pkg_name] = pkg
    return result


def _build_file_index(pkgs=_build_package_dict()):
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
    interfaces = file_index['interfaces']
    return interfaces



def list_interface_files(file_index=_build_file_index()):
    interfaces = file_index['interfaces']
    return [i['file'] for i in interfaces]


def list_providers(file_index=_build_file_index()):
    providers = file_index['providers']
    return providers



def list_interface_files(file_index=_build_file_index()):
    providers = file_index['providers']
    return [i['file'] for i in providers]



def list_semantic_interfaces(file_index=_build_file_index()):
    semantic_interfaces = file_index['semantic_interfaces']
    return semantic_interfaces


def list_semantic_interface_files(file_index=_build_file_index()):
    semantic_interfaces = file_index['semantic_interfaces']
    return [i['file'] for i in semantic_interfaces]
