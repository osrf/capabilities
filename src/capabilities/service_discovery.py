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

"""This module provides a way to create a spec_index remotely via a ROS service call
"""

import rospy

from capabilities.srv import GetCapabilitySpecs

from capabilities.specs.interface import capability_interface_from_string

from capabilities.specs.provider import capability_provider_from_string

from capabilities.specs.semantic_interface import semantic_capability_interface_from_string

from capabilities.discovery import _spec_loader


def spec_index_from_service(server_node_name='capability_server'):
    """Builds a :py:class:`SpecIndex` by calling a ROS service to get the specs

    Works just like :py:func:`spec_index_from_spec_file_index`, except the raw
    spec files are retreived over a service call rather than from disk.

    :param server_node_name: Name of the capability server's node,
        default is 'capability_server'
    :type server_node_name: str

    :raises: :py:class:`rospy.ServiceException` when the service call fails
    """
    service_name = '/{0}/get_capability_specs'.format(server_node_name)
    rospy.wait_for_service(service_name)
    get_capability_specs = rospy.ServiceProxy(service_name, GetCapabilitySpecs)
    response = get_capability_specs()
    spec_raw_index = {}
    for spec in response.capability_specs:
        package_dict = spec_raw_index.get(spec.package, {
            'capability_interface': [],
            'semantic_capability_interface': [],
            'capability_provider': []
        })
        if spec.type in ['capability_interface', 'semantic_capability_interface']:
            package_dict[spec.type].append((spec.content, spec.default_provider))
        else:
            package_dict[spec.type].append(spec.content)
        spec_raw_index[spec.package] = package_dict

    def capability_interface_loader(interface_tuple, package_name, spec_index):
        raw, default_provider = interface_tuple
        interface = capability_interface_from_string(raw)
        interface.default_provider = default_provider
        spec_index.add_interface(interface, 'service call', package_name)

    def semantic_capability_loader(interface_tuple, package_name, spec_index):
        raw, default_provider = interface_tuple
        si = semantic_capability_interface_from_string(raw)
        si.default_provider = default_provider
        spec_index.add_semantic_interface(si, 'service call', package_name)

    def capability_provider_loader(raw, package_name, spec_index):
        provider = capability_provider_from_string(raw)
        spec_index.add_provider(provider, 'service call', package_name)

    return _spec_loader(spec_raw_index, {
        'capability_interface': capability_interface_loader,
        'semantic_capability_interface': semantic_capability_loader,
        'capability_provider': capability_provider_loader
    })
