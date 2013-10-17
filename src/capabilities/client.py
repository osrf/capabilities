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

"""Provides a simple Python interface for interacting with the capability server

Typical usage::

    >>> from capabilities.client import Client
    >>> client = Client(capability_server_node_name='/capability_server_node_name')
    >>> client.use_capability('foo_pkg/Foo')
    >>> client.use_capability('foo_pkg/Foo')
    >>> client.free_capability('foo_pkg/Foo')
    >>> client.shutdown()

"""

import rospy

from bondpy import Bond

from capabilities.srv import FreeCapability
from capabilities.srv import UseCapability


class CapabilitiesClient(object):
    """Single point of entry for interacting with a remote capability server

    :param capability_server_node_name: name of the remote capability server
    :type capability_server_node_name: str
    """
    def __init__(self, capability_server_node_name='/capability_server'):
        self._name = capability_server_node_name
        self._bonds = []
        # Create service proxy for free_capability
        service_name = '{0}/free_capability'.format(capability_server_node_name)
        self.__free_capability = rospy.ServiceProxy(service_name, FreeCapability)
        # Create service proxy for use_capability
        service_name = '{0}/use_capability'.format(capability_server_node_name)
        self.__use_capability = rospy.ServiceProxy(service_name, UseCapability)

    def __wait_for_service(self, service, timeout):
        if not service.wait_for_service(timeout):
            rospy.logwarn("Timed out after waiting '{0}' seconds for service '{1}' to be available."
                          .format(timeout, service.resolved_name))
            return False
        return True

    def free_capability(self, capability_interface, timeout=None):
        """Free's a previously used capability.

        Calls the ~free_capability service, and closes the bond with that
        topic if the reference count is zero.

        :param capability_interface: Name of the capability interface to free up
        :type capability_interface: str
        :param timeout: time to wait on service to be available (optional)
        :type timeout: rospy.Duration
        :returns: True if successful, otherwise False
        :rtype: bool
        """
        if capability_interface not in self._bonds:
            rospy.logerr("Cannot free capability interface '{0}', because it was not first used."
                         .format(capability_interface))
            return False
        if not self.__wait_for_service(self.__free_capability, timeout):
            return False
        bond = self._bonds[capability_interface].pop()
        self.__free_capability.call(capability_interface, bond.id)
        bond.break_bond()
        return True

    def shutdown(self):
        """Cleanly frees any used capabilities and bonds."""
        for bonds in self._bonds.values():
            for bond in bonds:
                bond.break_bond()

    def use_capability(self, capability_interface, preferred_provider=None, timeout=None):
        """Declares that this capability is being used.

        Calls the `~use_capability` service, and opens a bond with that topic.
        This way the capability is started if it has not been already, and the
        internal reference count for the capability server is incremented.
        If the bond fails (this program crashes) then the reference is decremented.
        The reference is also decremented if free_capability is called.

        :param capability_interface: Name of the capability interface to use
        :type capability_interface: str
        :param preferred_provider: preferred provider or None for default provider (optional)
        :type preferred_provider: str
        :param timeout: time to wait on service to be available (optional)
        :type timeout: rospy.Duration
        :returns: True if successful, otherwise False
        :rtype: bool
        """
        if not self.__wait_for_service(self.__use_capability, timeout):
            return False
        resp = self.__use_capability.call(capability_interface, preferred_provider or '')
        if not resp.bond_id:
            # The service call failed
            return False
        if capability_interface not in self._bonds:
            self._bonds[capability_interface] = []
        self._bonds[capability_interface].append(Bond("{0}/bonds".format(self._name), resp.bond_id))
        self._bonds[capability_interface][-1].start()
        return True
