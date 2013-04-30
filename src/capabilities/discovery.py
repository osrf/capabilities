# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Tully Foote <tfoote@osrfoundation.org>

"""
This module implements discovery of packages which export various spec files.
"""

from __future__ import print_function

import os

from catkin_pkg.packages import find_packages


def _get_ros_package_paths():
    return os.getenv('ROS_PACKAGE_PATH', '').split(':')


def _build_package_dict(ros_package_path=None):
    ros_package_path = _get_ros_package_paths() if ros_package_path is None else ros_package_path
    result = {}
    for path in ros_package_path:
        for pkg_name, pkg in find_packages(path).items():
            result[os.path.abspath(os.path.join(path, pkg_name))] = pkg
    return result


def _build_file_index(pkgs=None):
    pkgs = _build_package_dict() if pkgs is None else pkgs
    interfaces = {}
    providers = {}
    semantic_interfaces = {}
    for pkg_name, pkg in pkgs.items():
        for e in pkg.exports:
            t = e.tagname
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


def list_interfaces(file_index=None):
    file_index = _build_file_index() if file_index is None else file_index
    interfaces = file_index['interfaces']
    return interfaces


def list_interface_files(file_index=None):
    file_index = _build_file_index() if file_index is None else file_index
    interfaces = file_index['interfaces']
    result = []
    for p, v in interfaces.items():
        result.extend([os.path.join(p, payload['file']) for payload in v])
    return result


def list_providers(file_index=None):
    file_index = _build_file_index() if file_index is None else file_index
    providers = file_index['providers']
    return providers


def list_provider_files(file_index=None):
    file_index = _build_file_index() if file_index is None else file_index
    providers = file_index['providers']
    result = []
    for p, v in providers.items():
        result.extend([os.path.join(p, payload['file']) for payload in v])
    return result


def list_semantic_interfaces(file_index=None):
    file_index = _build_file_index() if file_index is None else file_index
    semantic_interfaces = file_index['semantic_interfaces']
    return semantic_interfaces


def list_semantic_interface_files(file_index=None):
    file_index = _build_file_index() if file_index is None else file_index
    semantic_interfaces = file_index['semantic_interfaces']
    result = []
    for p, v in semantic_interfaces.items():
        result.extend([os.path.join(p, payload['file']) for payload in v])
    return result
