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

"""Provides a simple Python interface for interacting with the capability server.

Typical usage::

    >>> from capabilities.client import Client
    >>> client = Client()
    >>> # Use the line below if the capability_server has a different name
    >>> # client = Client(capability_server_node_name='/capability_server_node_name')
    >>> if not client.wait_for_services(3.0):  # Wait upto 3.0 seconds for the required ROS services
    ...     import sys
    ...     sys.exit("capability_server, called '{0}', failed to come up.".format(client._name))
    >>> client.use_capability('foo_pkg/Foo')
    >>> client.use_capability('foo_pkg/Foo')
    >>> client.free_capability('foo_pkg/Foo')
    >>> client.shutdown()

"""

import atexit

import rospy

from bondpy.bondpy import Bond

from capabilities.srv import EstablishBond
from capabilities.srv import FreeCapability
from capabilities.srv import UseCapability


class ServiceNotAvailableException(Exception):
    def __init__(self, service_name):
        Exception.__init__(self, "Required ROS service '{0}' not available"
                                 .format(service_name))


class CannotEstablishBondException(Exception):
    def __init__(self):  # pragma: no cover
        Exception.__init__(self, "Failed to establish bond.")


class CapabilityNotRunningException(Exception):
    def __init__(self, interface):
        Exception.__init__(self, "Capability interface '{0}' is not running."
                                 .format(interface))


class CapabilityNotInUseException(Exception):
    def __init__(self, interface):
        Exception.__init__(self, "Capability interface '{0}' not previously used."
                                 .format(interface))


class CapabilitiesClient(object):
    """Single point of entry for interacting with a remote capability server.

    Instantiation of this class does not check to see if the underlying
    services are available, call :py:meth:`wait_for_services` if you want
    to ensure that the services are available before continuing.

    :param capability_server_node_name: name of the remote capability server
    :type capability_server_node_name: str
    """
    def __init__(self, capability_server_node_name='/capability_server'):
        self._name = capability_server_node_name
        self._bond = None
        self._bond_id = None
        self._used_capabilities = set([])
        self._services = {}
        # Create service proxy for establish_bond
        service_name = '{0}/establish_bond'.format(capability_server_node_name)
        self.__establish_bond = rospy.ServiceProxy(service_name, EstablishBond)
        self._services['establish_bond'] = self.__establish_bond
        # Create service proxy for free_capability
        service_name = '{0}/free_capability'.format(capability_server_node_name)
        self.__free_capability = rospy.ServiceProxy(service_name, FreeCapability)
        self._services['free_capability'] = self.__free_capability
        # Create service proxy for use_capability
        service_name = '{0}/use_capability'.format(capability_server_node_name)
        self.__use_capability = rospy.ServiceProxy(service_name, UseCapability)
        self._services['use_capability'] = self.__use_capability
        # Register atexit shutdown
        atexit.register(self.shutdown)

    def wait_for_services(self, timeout=None, services=None):
        """Blocks until the requested services are available.

        Services are waited on one at a time. Therefore, if a non-None value
        for timeout is given, and one or more services are being waited on,
        the full timeout will be given to each wait, meaning that the total
        run time of this function can exceed the timeout provided.

        :param timeout: Seconds to wait for the services before returning False.
            If None is passed, then this will block indefinitely until the
            services are available.
        :type timeout: float
        :param services: List of services to wait on.
            If None is passed, then this will check all the services.
        :type services: list
        :returns: :py:obj:`True` is the services are available, :py:obj:`False` otherwise (timeout)
        :rtype: :py:obj:`bool`
        """
        services = self._services.keys() if services is None else services
        assert isinstance(services, list), services
        for service in services:
            if service not in self._services:
                raise ValueError(
                    "Service '{0}' is not a valid service and cannot be waited on. These are the valid services: {1}"
                    .format(service, list(self._services.keys())))
            if self.__wait_for_service(self._services[service], timeout) is False:
                return False
        return True

    def __wait_for_service(self, service, timeout):
        try:
            service.wait_for_service(timeout)
        except rospy.ROSException:
            rospy.logwarn("Timed out after waiting '{0}' seconds for service '{1}' to be available."
                          .format(timeout, service.resolved_name))
            return False
        return True

    def establish_bond(self, timeout=None):
        """Establishes a bond using the ``~establish_bond`` service call

        The bond id which is returned by the service call is stored internally
        and used implicitly by the use/free capabilities functions.

        If :py:meth:`establish_bond` had previously been called, then the old bond will be
        broken, which will result in any capability uses under that bond to be
        implicitly freed.

        This function is called implicitly by :py:meth:`use_capability` if
        no bond exists.

        This function will block waiting for the service call to become
        available and to wait for the bond to be established. In both cases
        the timeout parameter is used.

        None is returned if the timeout occurs while waiting on the service
        to become available or while waiting for the bond to form.

        :param timeout: time in seconds to wait for the service to be available
        :type timeout: float
        :returns: the bond_id received from the server or :py:obj:`None` on failure
        :rtype: :py:obj:`str`
        """
        if self.__wait_for_service(self.__establish_bond, timeout) is False:
            return None
        resp = self.__establish_bond()
        if not resp.bond_id:  # pragma: no cover
            return None
        self._bond_id = resp.bond_id
        self._bond = Bond('{0}/bonds'.format(self._name), self._bond_id)
        self._bond.start()
        timeout_dur = None if timeout is None else rospy.Duration.from_sec(timeout)
        if self._bond.wait_until_formed(timeout_dur) is False:  # pragma: no cover
            return None
        return self._bond_id

    def free_capability(self, capability_interface, timeout=None):
        """Free's a previously used capability.

        Calls the ``~free_capability`` service, which effectively decrements
        the internal reference count for that capability in the remote
        capability server.

        If that results in a reference count of zero,
        then the capability server will shutdown that capability automatically.

        :param capability_interface: Name of the capability interface to free up
        :type capability_interface: str
        :param timeout: time to wait on service to be available (optional)
        :type timeout: float
        :returns: :py:obj:`True` if successful, otherwise :py:obj:`False`
        :rtype: :py:obj:`bool`
        :raises: CapabilityNotInUseException if the capability has not been previously used
        :raises: CapabilityNotRunningException if the capability has already been stopped
            which can be caused by a capability it depends on stopping
        :raises: ServiceNotAvailableException if the required service is not available
            after the timeout has expired
        """
        if capability_interface not in self._used_capabilities:
            rospy.logerr("Cannot free capability interface '{0}', because it was not first used."
                         .format(capability_interface))
            CapabilityNotInUseException(capability_interface)
        if self.__wait_for_service(self.__free_capability, timeout) is False:
            raise ServiceNotAvailableException(self.__free_capability.resolved_name)
        try:
            self.__free_capability.call(capability_interface, self._bond_id)
        except rospy.ServiceException as exc:
            exc_str = "{0}".format(exc)
            if 'Cannot free Capability' in exc_str and 'because it is not running' in exc_str:
                raise CapabilityNotRunningException(capability_interface)
        return True

    def shutdown(self):
        """Cleanly frees any used capabilities."""
        if self._bond:
            self._bond.break_bond()

    def use_capability(self, capability_interface, preferred_provider=None, timeout=None):
        """Declares that this capability is being used.

        Calls the ``~use_capability`` service, and opens a bond with capability server if none exists.
        This will cause the capability to be started, if it has not been already, and the
        internal reference count for that capability in the capability server is incremented.

        If the bond fails (i.e. this program crashes) then the reference is decremented automatically.
        The reference is also decremented if :py:meth:`free_capability` is called.

        :param capability_interface: Name of the capability interface to use
        :type capability_interface: str
        :param preferred_provider: preferred provider or None for default provider (optional)
        :type preferred_provider: str
        :param timeout: time to wait on service to be available (optional)
        :type timeout: float
        :returns: :py:obj:`True` if successful, otherwise :py:obj:`False`
        :rtype: :py:obj:`bool`
        :raises: ServiceNotAvailableException if the required service is not available
            after the timeout has expired
        :raise: CannotEstablishBondException is the bond with the capability_server cannot be established
        """
        # If no bond has been established, establish one first
        if self._bond is None:
            if self.establish_bond(timeout) is None:  # pragma: no cover
                raise CannotEstablishBondException()
        if self.__wait_for_service(self.__use_capability, timeout) is False:  # pragma: no cover
            raise ServiceNotAvailableException(self.__use_capability.resolved_name)
        self.__use_capability.call(capability_interface, preferred_provider or '', self._bond_id)
        self._used_capabilities.add(capability_interface)
        return True
