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

# Author: William Woodall <william@osrfoundation.org>

"""
This module implements the Capability server.

The Capability server provides access to queries and services related
to capabilities.
"""

from __future__ import print_function

import argparse
import os
import sys

from discovery import _build_package_dict, _build_file_index,\
    list_interface_files, list_provider_files, list_semantic_interface_files

from capabilities.specs.interface import capability_interface_from_file_path
from capabilities.specs.provider import capability_provider_from_file_path
from capabilities.specs.semantic_interface\
    import semantic_capability_interface_from_file_path


class CapabilityServer(object):
    def __init__(self, ros_package_path=None):
        self._interfaces = {}
        self._providers = {}
        self._semantic_interfaces = {}

        self.load_from_ros_package_path(ros_package_path)
        # TODO verify
        # TODO setup service

    def load_from_ros_package_path(self, ros_package_path=None):
        """
        Run discovery, possible to reexecute
        assert there are no duplicate names
        """
        pkgs = _build_package_dict(ros_package_path)
        capabilities = _build_file_index(pkgs)
        for i in list_interface_files(capabilities):
            interface = capability_interface_from_file_path(i)
            if not self._check_name(interface.name, 'interface'):
                continue
            self._interfaces[interface.name] = interface
        for i in list_provider_files(capabilities):
            provider = capability_provider_from_file_path(i)
            if not self._check_name(provider.name, 'provider'):
                continue
            self._providers[provider.name] = provider
        for i in list_semantic_interface_files(capabilities):
            semantic_interface = semantic_capability_interface_from_file_path(i)
            if not self._check_name(semantic_interface.name, 'semantic_interface'):
                continue
            self._semantic_interfaces[semantic_interface.name] = semantic_interface

    def _check_name(self, name, type_name):
        error = False
        if name in self._interfaces:
            print("%s %s already declared" % \
                      (type_name, name))
            error = True
        elif name in self._providers:
            print("%s %s collides with a provider name" % \
                      (type_name, name))
            error = True
        elif name in self._semantic_interfaces:
            print("%s %s collides with a semantic_interface name" % \
                      (type_name, name))
            error = True
        return error == False

    def verify_tree(self):
        """" Check the tree for issues
        typos, no name, cross reference errors.
        """
        raise NotImplemented

    def view_capabilities_as_dot(self, with_errors=False):
        """
        View the system, optionally with errors.
        """
        raise NotImplemented

    def advertize_services(self):
        """
        advertize the public API
        """
        raise NotImplemented

    def get_interfaces(self):
        return self._interfaces

    def get_providers(self, interface_name):
        return [p for p in self._providers.values() if p.implements == interface_name]
    
    def get_semantic_interfaces(self, interface_name):
        return [p for p in self._semantic_interfaces.values() if p.redefines == interface_name]

"""
Advertized services

reload_capabilites

get_interfaces

get_providers(interface)

get_semantic_interfaces(interface)


launching
---------

run_capability(interface, (preferred provider) )

"""


def create_parser():
    parser = argparse.ArgumentParser(description="Runs the capability server")
    add = parser.add_argument
    add('package_path', nargs='?', help="Overrides the ROS_PACKAGE_PATH when discovering packages with capabilities")
    return parser


def main(sysargv=None):
    parser = create_parser()
    args = parser.parse_args(sysargv)

    ros_package_path = args.package_path or os.environ.get('ROS_PACKAGE_PATH', '')
    ros_package_path = [x for x in ros_package_path.split(':') if x]
    if not ros_package_path:
        sys.exit('No package paths specified, set ROS_PACKAGE_PATH or pass them as an argument')

    # TODO: find capabilities, and start service interface
    cs = CapabilityServer(ros_package_path)


    print("Capability server created.\ninterfaces: %s\nproviders%s\nsemantic_interfaces:%s" % 
          (cs._interfaces, cs._providers, cs._semantic_interfaces))

    print("interfaces are", cs.get_interfaces)

    print("providers of interface Minimal are", cs.get_providers('Minimal'))

    print("semantic_interfaces of interface Minimal are",
          cs.get_semantic_interfaces('Minimal'))
