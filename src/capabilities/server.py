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

import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from capabilities.srv import GetInterfaces
from capabilities.srv import GetInterfacesResponse
from capabilities.srv import GetProviders
from capabilities.srv import GetProvidersResponse
from capabilities.srv import GetSemanticInterfaces
from capabilities.srv import GetSemanticInterfacesResponse

from capabilities.discovery import package_index_from_package_path
from capabilities.discovery import spec_file_index_from_package_index
from capabilities.discovery import spec_index_from_spec_file_index
from capabilities.discovery import DuplicateNameException
from capabilities.discovery import InterfaceNameNotFoundException


class CapabilityServer(object):
    """A class to expose the :py:class:`discovery.SpecIndex` over a ROS API
    """

    def __init__(self, package_paths):
        self.__package_paths = package_paths
        self.__spec_index = None

    def run(self):
        self.__load_capabilities()

        self.__reload_service = rospy.Service(
            'reload_capabilities', Empty, self.handle_reload_request)

        self.__interfaces_service = rospy.Service(
            'get_interfaces', GetInterfaces, self.handle_get_interfaces)

        self.__providers_service = rospy.Service(
            'get_providers', GetProviders, self.handle_get_providers)

        self.__semantic_interfaces_service = rospy.Service(
            'get_semantic_interfaces', GetSemanticInterfaces,
            self.handle_get_semantic_interfaces)

        rospy.loginfo("Capability Server Ready")

        rospy.spin()

    def __load_capabilities(self):
        package_index = package_index_from_package_path(self.__package_paths)
        spec_file_index = spec_file_index_from_package_index(package_index)
        spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
        if errors:
            rospy.logerror("Errors were encountered while loading capabilities:")
            for error in errors:
                if type(error) == DuplicateNameException:
                    rospy.logerror("  DuplicateNameException: " + str(error))
                elif type(error) == InterfaceNameNotFoundException:
                    rospy.logerror("  InterfaceNameNotFoundException: " + str(error))
                else:
                    rospy.logerror("  " + str(type(error)) + ": " + str(error))
        self.__spec_index = spec_index

    def handle_reload_request(self, req):
        rospy.loginfo("Reloading capabilities...")
        self.__load_capabilities()
        return EmptyResponse()

    def handle_get_interfaces(self, req):
        return GetInterfacesResponse(self.__spec_index.interface_names)

    def handle_get_providers(self, req):
        if req.interface:
            providers = [p.name
                         for p in self.__spec_index.providers.values()
                         if p.implements == req.interface]
        else:
            providers = self.__spec_index.provider_names
        return GetProvidersResponse(providers)

    def handle_get_semantic_interfaces(self, req):
        if req.interface:
            sifaces = [si.name
                       for si in self.__spec_index.semantic_interfaces.values()
                       if si.redefines == req.interface]
        else:
            sifaces = self.__spec_index.semantic_interface_names
        return GetSemanticInterfacesResponse(sifaces)


def create_parser():
    parser = argparse.ArgumentParser(description="Runs the capability server")
    add = parser.add_argument
    add('package_path', nargs='?',
        help="Overrides ROS_PACKAGE_PATH when discovering capabilities")
    return parser


def main(sysargv=None):
    parser = create_parser()
    args = parser.parse_args(sysargv)

    ros_package_path = args.package_path or os.getenv('ROS_PACKAGE_PATH', '')
    ros_package_path = [x for x in ros_package_path.split(':') if x]
    if not ros_package_path:
        sys.exit('No package paths specified, set ROS_PACKAGE_PATH or pass them as an argument')

    rospy.init_node('capability_server')

    capability_server = CapabilityServer(ros_package_path)
    capability_server.run()
