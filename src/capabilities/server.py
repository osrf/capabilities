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
import logging
import os
import sys
import threading
import traceback
import uuid

import rospy

from bondpy.bondpy import Bond

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from capabilities.srv import EstablishBond
from capabilities.srv import EstablishBondResponse
from capabilities.srv import GetCapabilitySpec
from capabilities.srv import GetCapabilitySpecResponse
from capabilities.srv import FreeCapability
from capabilities.srv import FreeCapabilityResponse
from capabilities.srv import GetCapabilitySpecs
from capabilities.srv import GetCapabilitySpecsResponse
from capabilities.srv import GetInterfaces
from capabilities.srv import GetInterfacesResponse
from capabilities.srv import GetNodeletManagerName
from capabilities.srv import GetNodeletManagerNameResponse
from capabilities.srv import GetProviders
from capabilities.srv import GetProvidersResponse
from capabilities.srv import GetSemanticInterfaces
from capabilities.srv import GetSemanticInterfacesResponse
from capabilities.srv import GetRemappings
from capabilities.srv import GetRemappingsResponse
from capabilities.srv import GetRunningCapabilities
from capabilities.srv import GetRunningCapabilitiesResponse
from capabilities.srv import StartCapability
from capabilities.srv import StartCapabilityResponse
from capabilities.srv import StopCapability
from capabilities.srv import StopCapabilityResponse
from capabilities.srv import UseCapability
from capabilities.srv import UseCapabilityResponse

from capabilities.discovery import package_index_from_package_path
from capabilities.discovery import spec_file_index_from_package_index
from capabilities.discovery import spec_index_from_spec_file_index

from capabilities.launch_manager import _special_nodelet_manager_capability
from capabilities.launch_manager import LaunchManager

from capabilities.msg import Capability
from capabilities.msg import CapabilityEvent
from capabilities.msg import CapabilitySpec
from capabilities.msg import Remapping
from capabilities.msg import RunningCapability

from capabilities.specs.interface import capability_interface_from_string
from capabilities.specs.semantic_interface import semantic_capability_interface_from_string

USER_SERVICE_REASON = 'user service call'

## Hack to squelch output from Service call failure ##

from rospy.impl import tcpros_service


def custom__handle_request(self, transport, request):  # pragma: no cover
    import struct
    from rospy.impl.tcpros_service import convert_return_to_response
    from rospy.service import ServiceException
    try:
        # convert return type to response Message instance
        response = convert_return_to_response(self.handler(request), self.response_class)
        self.seq += 1
        # ok byte
        transport.write_buff.write(struct.pack('<B', 1))
        transport.send_message(response, self.seq)
    except ServiceException as e:
        rospy.core.rospydebug("handler raised ServiceException: %s" % e)
        self._write_service_error(transport, "service cannot process request: %s" % e)
    except Exception as e:
        # rospy.logerr("Error processing request: %s\n%s" % (e, traceback.print_exc()))
        self._write_service_error(transport, "error processing request: %s" % e)

tcpros_service.ServiceImpl._handle_request = custom__handle_request

## End hacks ##


class CapabilityInstance(object):
    """Encapsulates the state of an instance of a Capability Provider

    This class encapsulates the state of the capability instance and
    provides methods for changing the states of the instance.
    """
    def __init__(self, provider, provider_path, started_by='unknown'):
        self.__state = 'waiting'
        self.name = provider.name
        self.provider = provider
        self.provider_path = provider_path
        self.interface = provider.implements
        self.pid = None
        self.depends_on = [x for x in provider.dependencies]
        self.canceled = False
        self.started_by = started_by
        self.bonds = {}  # {bond_id: reference_count}

    @property
    def reference_count(self):
        return sum(list(self.bonds.values()))

    @property
    def state(self):
        """Get the current state of the CapabilityInstance"""
        return self.__state

    def launch(self):
        """Change to the 'launching' state

        Fails to transition if the current state is not 'waiting'.

        :returns: True if transition is successful, False otherwise
        :rtype: bool
        """
        if self.state != 'waiting':
            rospy.logerr(
                "Capability Provider '{0}' ".format(self.name) +
                "cannot transition to 'launching' from anything but " +
                "'waiting', current state is '{0}'".format(self.state))
            return False
        self.__state = 'launching'
        return True

    def cancel(self):
        """Cancels the instance, which can only be done while it is still 'waiting'

        Fails to cancel if the current state is not 'waiting'.
        "Canceling" is achieved by setting the canceled member variable to True.

        :returns: True if canceling is successful, False otherwise
        :rtype: bool
        """
        if self.state != 'waiting':
            rospy.logerr(
                "Capability Instance '{0}' ".format(self.name) +
                "cannot be canceled from anything but " +
                "'waiting', current state is '{0}'".format(self.state))
            return False
        self.canceled = True
        return True

    def launched(self, pid):
        """Called once the instance is "launched", changes state to 'running'

        Fails to transition if the current state is not 'launching'.
        If successful, the state changes to 'running'.

        :param pid: process ID of the instance being tracked
        :type pid: int
        :returns: True if transition is successful, False otherwise
        :rtype: bool
        """
        self.pid = pid
        if self.state != 'launching':
            rospy.logerr(
                "Capability Instance '{0}' ".format(self.name) +
                "cannot transition to 'running' from anything but " +
                "'launching', current state is '{0}'".format(self.state))
            return False
        self.__state = 'running'
        return True

    def stopped(self):
        """Change to the 'stopping' state

        Fails to transition if the current state is not either 'running' or 'launching'.

        :returns: True if transition is successful, False otherwise
        :rtype: bool
        """
        if self.state not in ['running', 'launching']:
            rospy.logerr(
                "Capability Instance '{0}' ".format(self.name) +
                "cannot transition to 'stopping' from anything but " +
                "'launching' or 'running', " +
                "current state is '{0}'".format(self.state))
            return False
        self.__state = 'stopping'
        return True

    def terminated(self):
        """Called when the instance has terminated, transitions to the 'terminated' state

        Fails to transition if the current state is not 'stopping'.

        :returns: True if transition is successful, False otherwise
        :rtype: bool
        """
        result = True
        if self.state != 'stopping':
            rospy.logerr(
                "Capability Instance '{0}' ".format(self.name) +
                "terminated unexpectedly, it was previously in the " +
                "'{0}' state.".format(self.state))
            result = False
        self.__state = 'terminated'
        return result


def get_reverse_depends(name, capability_instances):
    """Gets the reverse dependencies of a given Capability

    :param name: Name of the Capability which the instances might depend on
    :type name: str
    :param capability_instances: list of instances to search for having a
        dependency on the given Capability
    :type capability_instances: :py:obj:`list` of :py:class:`CapabilityInstance`
    :returns: A list of :py:class:`CapabilityInstance`'s which depend on the
        given Capability name
    :rtype: :py:obj:`list` of :py:class:`CapabilityInstance`
    """
    rdepends = []
    for instance in capability_instances:
        if name in instance.depends_on:
            rdepends.append(instance)
    return rdepends


class CapabilityServer(object):
    """A class to expose the :py:class:`discovery.SpecIndex` over a ROS API
    """

    def __init__(self, package_paths, screen=None):
        self.__package_paths = package_paths
        self.__spec_index = None
        self.__graph_lock = threading.Lock()
        self.__capability_instances = {}
        self.__launch_manager = LaunchManager(
            screen=bool(rospy.get_param('~use_screen', screen)),
            nodelet_manager_name=rospy.get_param('~nodelet_manager_name', None)
        )
        self.__debug = False
        self.__package_whitelist = None
        self.__package_blacklist = None
        self.__whitelist = None
        self.__blacklist = None
        self.__default_providers = {}
        self.__missing_default_provider_is_an_error = rospy.get_param('~missing_default_provider_is_an_error', False)
        self.__bonds = {}

    def spin(self):
        """Starts the capability server by setting up ROS comms, then spins"""
        self.__package_whitelist = rospy.get_param('~package_whitelist', None)
        if not isinstance(self.__package_whitelist, (list, tuple, type(None))):
            msg = "~package_whitelist must be a list or null, got a '{0}'".format(type(self.__whitelist))
            rospy.logerr(msg)
            self.__package_whitelist = None
        self.__package_blacklist = rospy.get_param('~package_blacklist', None)
        if not isinstance(self.__package_blacklist, (list, tuple, type(None))):
            msg = "~package_blacklist must be a list or null, got a '{0}'".format(type(self.__whitelist))
            rospy.logerr(msg)
            self.__package_blacklist = None
        self.__whitelist = rospy.get_param('~whitelist', None)
        if not isinstance(self.__whitelist, (list, tuple, type(None))):
            msg = "~whitelist must be a list or null, got a '{0}'".format(type(self.__whitelist))
            rospy.logerr(msg)
            self.__whitelist = None
        self.__blacklist = rospy.get_param('~blacklist', None)
        if not isinstance(self.__blacklist, (list, tuple, type(None))):
            msg = "~blacklist must be a list or null, got a '{0}'".format(type(self.__blacklist))
            rospy.logerr(msg)
            self.__blacklist = None
        self.__debug = rospy.get_param('~debug', False)
        if self.__debug:
            logger = logging.getLogger('rosout')
            logger.setLevel(logging.DEBUG)
            rospy.logdebug('Debug messages enabled.')

        self.__load_capabilities()

        self.__bond_topic = rospy.get_name() + "/bonds"

        # Collect default arguments
        self.__populate_default_providers()

        self.__start_capability_service = rospy.Service(
            '~start_capability', StartCapability, self.handle_start_capability)

        self.__stop_capability_service = rospy.Service(
            '~stop_capability', StopCapability, self.handle_stop_capability)

        self.__establish_bond_service = rospy.Service(
            '~establish_bond', EstablishBond, self.handle_establish_bond)

        self.__free_capability_service = rospy.Service(
            '~free_capability', FreeCapability, self.handle_free_capability)

        self.__use_capability_service = rospy.Service(
            '~use_capability', UseCapability, self.handle_use_capability)

        self.__reload_service = rospy.Service(
            '~reload_capabilities', Empty, self.handle_reload_request)

        self.__interfaces_service = rospy.Service(
            '~get_interfaces', GetInterfaces, self.handle_get_interfaces)

        self.__providers_service = rospy.Service(
            '~get_providers', GetProviders, self.handle_get_providers)

        self.__semantic_interfaces_service = rospy.Service(
            '~get_semantic_interfaces', GetSemanticInterfaces,
            self.handle_get_semantic_interfaces)

        self.__running_capabilities = rospy.Service(
            '~get_running_capabilities', GetRunningCapabilities,
            self.handle_get_running_capabilities)

        self.__capability_specs = rospy.Service(
            '~get_capability_specs', GetCapabilitySpecs,
            self.handle_get_capability_specs)

        self.__capability_spec = rospy.Service(
            '~get_capability_spec', GetCapabilitySpec,
            self.handle_get_capability_spec)

        self.__get_nodelet_manager_name_service = rospy.Service(
            '~get_nodelet_manager_name', GetNodeletManagerName,
            self.handle_get_nodelet_manager_name)

        self.__get_remappings_service = rospy.Service(
            '~get_remappings', GetRemappings,
            self.handle_get_remappings)

        rospy.Subscriber(
            '~events', CapabilityEvent, self.handle_capability_events)

        rospy.loginfo("Capability Server Ready")
        rospy.Publisher("~events", CapabilityEvent, queue_size=1000).publish(
            CapabilityEvent(type=CapabilityEvent.SERVER_READY))

        rospy.spin()

    def shutdown(self):
        """Stops the capability server and cleans up any running processes"""
        for instance in self.__capability_instances.values():  # pragma: no cover
            if instance.state in ['running', 'launching']:
                instance.stopped()
            if instance.state == 'waiting':
                instance.cancel()
        self.__launch_manager.stop()

    def __load_capabilities(self):
        package_index = package_index_from_package_path(self.__package_paths)
        self.spec_file_index = spec_file_index_from_package_index(package_index)
        # Prune packages by black and white list
        for package in self.spec_file_index.keys():
            if self.__package_whitelist and package not in self.__package_whitelist:
                rospy.loginfo("Package '{0}' not in whitelist, skipping.".format(package))
                del self.spec_file_index[package]
            if self.__package_blacklist and package in self.__package_blacklist:
                rospy.loginfo("Package '{0}' in blacklist, skipping.".format(package))
                del self.spec_file_index[package]
        # Generate spec_index from spec file index
        spec_index, errors = spec_index_from_spec_file_index(self.spec_file_index)
        if errors:
            rospy.logerr("Errors were encountered loading capabilities:")
            for error in errors:
                rospy.logerr("  " + str(error.__class__.__name__) + ": " + str(error))
        # Prune specific capabilities based on black and white lists
        removed_interfaces = []
        for specs, remove_func in [
            (spec_index.interfaces, spec_index.remove_interface),
            (spec_index.semantic_interfaces, spec_index.remove_semantic_interface),
            (spec_index.providers, spec_index.remove_provider)
        ]:
            for spec in specs.keys():
                if self.__whitelist and spec not in self.__whitelist:
                    removed_interfaces.append(spec)
                    remove_func(spec)
                    rospy.loginfo("Spec '{0}' is not in the whitelist, skipping.".format(spec))
                if self.__blacklist and spec in self.__blacklist:
                    removed_interfaces.append(spec)
                    remove_func(spec)
                    rospy.loginfo("Spec '{0}' is in the blacklist, skipping.".format(spec))
        # Remove providers which no longer have an interface
        for interface in removed_interfaces:
            for provider in spec_index.providers.values():
                if provider.implements == interface:
                    spec_index.remove_provider(provider.name)
        self.__spec_index = spec_index

    def __populate_default_providers(self):
        # Collect available interfaces
        interfaces = self.__spec_index.interface_names + self.__spec_index.semantic_interface_names
        for interface in interfaces:
            # Collect the providers for each interface
            providers = [n
                         for n, p in self.__spec_index.providers.items()
                         if p.implements == interface]
            if not providers:
                # If an interface has not providers, ignore it
                rospy.logwarn("No providers for capability interface '{0}', not checking for default provider."
                              .format(interface))
                continue
            try:
                # Try to get the default provider from the corresponding ros parameter
                self.__default_providers[interface] = rospy.get_param('~defaults/' + interface)
            except KeyError:
                # No ros parameter set for this capability interface
                rospy.logwarn("No default provider given for capability interface '{0}'. ".format(interface))
                if len(providers) == 1:
                    # If there is only one provider, allow it to be the default
                    rospy.logwarn("'{0}' has only one provider, '{1}', using that as the default."
                                  .format(interface, providers[0]))
                    self.__default_providers[interface] = providers[0]
                else:
                    # Otherwise we can't decide
                    if self.__missing_default_provider_is_an_error:
                        rospy.logerr("Could not determine a default provider for capability interface '{0}', aborting."
                                     .format(interface))
                        sys.exit(-1)
                    else:
                        rospy.logwarn("Could not determine a default provider for capability interface '{0}'."
                                      .format(interface))
                        continue
            # Make sure the given default provider exists
            if self.__default_providers[interface] not in self.__spec_index.provider_names:
                rospy.logerr("Given default provider '{0}' for interface '{1}' does not exist."
                             .format(self.__default_providers[interface], interface))
                sys.exit(-1)
            # Make sure the given provider implements this interface
            if self.__default_providers[interface] not in providers:
                rospy.logerr("Given default provider '{0}' does not implment interface '{1}'."
                             .format(self.__default_providers[interface], interface))
                sys.exit(-1)
            # Update the interface object with the default provider
            iface = self.__spec_index.interfaces.get(
                interface,
                self.__spec_index.semantic_interfaces.get(interface, None))
            iface.default_provider = self.__default_providers[interface]
        # Summarize defaults
        if self.__default_providers:
            rospy.loginfo("For each available interface, the default provider:")
            for interface, provider in self.__default_providers.items():
                rospy.loginfo("'{0}'".format(interface))
                rospy.loginfo("  => '{0}'".format(provider))
                rospy.loginfo("")
        else:  # pragma: no cover
            rospy.logwarn("No runnable Capabilities loaded.")

    def __catch_and_log(self, func, *args, **kwargs):
        warning_level_exceptions = ['because it is not running']
        try:
            return func(*args, **kwargs)
        except Exception as exc:
            msg = "{0}".format(exc)
            log_func = rospy.logerr
            if [x for x in warning_level_exceptions if x in msg]:
                log_func = rospy.logwarn
            rospy.logdebug(traceback.format_exc())
            log_func('{0}: {1}'.format(exc.__class__.__name__, msg))
            raise

    def handle_capability_events(self, event):
        """Callback for handling messages (events) from the /events topic

        This callback only process events generated by this node.

        :param event: ROS message encapsulating an event
        :type event: :py:class:`capabilities.msgs.CapabilityEvent`
        """
        return self.__catch_and_log(self._handle_capability_events, event)

    def _handle_capability_events(self, event):
        # Ignore any publications which we did not send (external publishers)
        if event._connection_header['callerid'] != rospy.get_name():
            return  # pragma: no cover
        # Ignore the `server_ready` event
        if event.type == event.SERVER_READY:
            return
        # Specially handle the nodelet manager
        if event.capability == _special_nodelet_manager_capability:
            if event.type == event.TERMINATED:
                if not rospy.is_shutdown():
                    rospy.logerr("Capability server's nodelet manager terminated unexpectedly.")
                    self.shutdown()
            return
        # Update the capability
        capability = event.capability
        with self.__graph_lock:
            if capability not in self.__capability_instances.keys():
                rospy.logerr("Unknown capability instance: '{0}'"
                             .format(capability))
                return
            instance = self.__capability_instances[capability]
            if event.type == event.LAUNCHED:
                if instance.canceled:  # pragma: no cover
                    # This is defensive programming, it should not happen
                    self.__stop_capability(instance.name)
                else:
                    instance.launched(event.pid)
            elif event.type == event.TERMINATED:
                instance.terminated()
                rospy.loginfo(
                    "Capability Provider '{0}' for Capability '{1}' "
                    .format(event.provider, event.capability) +
                    "has terminated.")
            # Update the graph
            self.__update_graph()

    def __remove_terminated_capabilities(self):
        # collect all of the terminated capabilities
        terminated = [x
                      for x in self.__capability_instances.values()
                      if x.state == 'terminated']
        # Remove terminated instances
        for instance in terminated:
            del self.__capability_instances[instance.interface]
            # Shutdown unused capabilities
            self.__cleanup_graph()

    def __cleanup_graph(self):
        """Iterate over the running capabilities and shutdown ones which are no longer needed

        For each running capability, if it was not started by the user then look at who depends on it.
        If no other capabilities depend on it, then shut it down.
        """
        # Collect all running capabilities
        running_capabilities = [x
                                for x in self.__capability_instances.values()
                                if x.state == 'running']
        for cap in running_capabilities:
            if cap.started_by == USER_SERVICE_REASON:
                # Started by user, do not garbage collect this
                continue
            rdepends = get_reverse_depends(cap.interface, self.__capability_instances.values())
            if rdepends:
                # Someone depends on me, do not garbage collect this
                rospy.logdebug("Keeping the '{0}' provider of the '{1}' interface, ".format(cap.name, cap.interface) +
                               "because other running capabilities depend on it.")
                continue
            if cap.state == 'running':
                rospy.loginfo("Stopping the '{0}' provider of the '{1}' interface, because it has no dependents left."
                              .format(cap.name, cap.interface))
                self.__stop_capability(cap.interface)
            elif cap.state == 'waiting':  # pragma: no cover
                rospy.loginfo("Canceling the '{0}' provider of the '{1}' interface, because it has no dependents left."
                              .format(cap.name, cap.interface))
                cap.cancel()
            # Else the state is launching, stopping, or terminated
            # In which case launching will be caught on the next cleanup
            # and the latter two will get cleared out also.

    def __update_graph(self):
        # collect all of the waiting capabilities
        waiting = [x
                   for x in self.__capability_instances.values()
                   if x.state == 'waiting']
        # If any of the waiting have no blocking dependencies start them
        for instance in waiting:
            blocking_dependencies = []
            for dependency_name in instance.depends_on:
                if dependency_name not in self.__capability_instances:  # pragma: no cover
                    rospy.logerr(
                        "Inconsistent capability run graph, '{0}' depends on "
                        .format(instance.name) + "'{0}', ".format(dependency_name) +
                        "which is not in the list of capability instances.")
                    return
                dependency = self.__capability_instances[dependency_name]
                if dependency.state != 'running':
                    blocking_dependencies.append(dependency)
            if not blocking_dependencies:
                instance.launch()
                self.__launch_manager.run_capability_provider(
                    instance.provider, instance.provider_path
                )
        # Remove any terminated capabilities
        self.__remove_terminated_capabilities()

    def __stop_capability(self, name):
        if name not in self.__capability_instances:
            rospy.logerr("Inconsistent capability run graph, asked to stop " +
                         "capability '{0}', ".format(name) +
                         "which is not in the list of capability instances.")
            return
        capability = self.__capability_instances[name]
        rdepends = get_reverse_depends(name, self.__capability_instances.values())
        for cap in rdepends:
            if cap.state in ['stopping', 'terminated']:  # pragma: no cover
                # It is possible that this cap was stopped by another cap in this list
                # This is purely defensive
                continue
            rospy.loginfo(
                "Capability '{0}' being stopped because its dependency '{1}' is being stopped.".format(cap.name, name))
            self.__stop_capability(cap.interface)
        capability.stopped()
        self.__launch_manager.stop_capability_provider(capability.pid)

    def __get_provider_dependencies(self, provider):
        result = []
        for interface, dep in provider.dependencies.items():
            provider_name = dep.provider or self.__default_providers[interface]
            if provider_name not in self.__spec_index.providers:
                # This is the case where a provider depends on another interface,
                # but the preferred provider does not exist
                raise RuntimeError("Capability Provider '{0}' not found"
                                   .format(provider_name))
            dep_provider = self.__spec_index.providers[provider_name]
            result.append((dep_provider, provider.name))
        return result

    def __get_capability_instances_from_provider(self, provider):
        instances = []
        providers = [(provider, USER_SERVICE_REASON)]
        while providers:
            curr, reason = providers.pop()
            providers.extend(self.__get_provider_dependencies(curr))
            curr_path = self.__spec_index.provider_paths[curr.name]
            instances.append(CapabilityInstance(curr, curr_path, started_by=reason))
        return instances

    def __get_providers_for_interface(self, interface, allow_semantic=False):
        valid_interfaces = [interface]
        if allow_semantic:
            # Add semantic interfaces which redefine this one
            valid_interfaces.extend(
                [k for k, v in self.__spec_index.semantic_interfaces.items()
                 if v.redefines == interface]
            )
        providers = dict([(n, p)
                          for n, p in self.__spec_index.providers.items()
                          if p.implements in valid_interfaces])
        return providers  # Could be empty

    def __start_capability(self, capability, preferred_provider):
        if capability not in self.__spec_index.interfaces.keys() + self.__spec_index.semantic_interfaces.keys():
            raise RuntimeError("Capability '{0}' not found.".format(capability))
        # If no preferred provider is given, use the default
        preferred_provider = preferred_provider or self.__default_providers[capability]
        providers = self.__get_providers_for_interface(capability, allow_semantic=True)
        if preferred_provider not in providers:
            raise RuntimeError(
                "Capability Provider '{0}' not found for Capability '{1}'"
                .format(preferred_provider, capability))
        provider = providers[preferred_provider]
        instances = self.__get_capability_instances_from_provider(provider)
        with self.__graph_lock:
            for x in instances:
                if x.interface not in self.__capability_instances:
                    self.__capability_instances[x.interface] = x
            self.__update_graph()
        return True

    def handle_get_capability_specs(self, req):
        return self.__catch_and_log(self._handle_get_capability_specs, req)

    def _handle_get_capability_specs(self, req):
        rospy.loginfo("Servicing request for capability specs...")
        response = GetCapabilitySpecsResponse()
        for package_name, package_dict in self.spec_file_index.items():
            for spec_type in ['capability_interface', 'semantic_capability_interface', 'capability_provider']:
                for path in package_dict[spec_type]:
                    with open(path, 'r') as f:
                        raw = f.read()
                        default_provider = ''
                        # If a capability interface, try to lookup the default provider
                        iface = None
                        if spec_type == 'capability_interface':
                            iface = capability_interface_from_string(raw, path)
                        if spec_type == 'semantic_capability_interface':
                            iface = semantic_capability_interface_from_string(raw, path)
                        if spec_type in ['capability_interface', 'semantic_capability_interface']:
                            iface.name = '{package}/{name}'.format(package=package_name, name=iface.name)
                            if iface.name not in self.__default_providers:
                                default_provider = ''
                            else:
                                default_provider = self.__default_providers[iface.name]
                        cs = CapabilitySpec(package_name, spec_type, raw, default_provider)
                        response.capability_specs.append(cs)
        return response

    def handle_get_capability_spec(self, req):
        return self.__catch_and_log(self._handle_get_capability_spec, req)

    def _handle_get_capability_spec(self, req):
        rospy.loginfo("Servicing request for get capability spec '{0}'...".format(req.capability_spec))
        response = GetCapabilitySpecResponse()
        for package_name, package_dict in self.spec_file_index.items():
            for spec_type in ['capability_interface', 'semantic_capability_interface', 'capability_provider']:
                for path in package_dict[spec_type]:
                    with open(path, 'r') as f:
                        raw = f.read()
                        default_provider = ''
                        # If a capability interface, try to lookup the default provider
                        iface = None
                        if spec_type == 'capability_interface':
                            iface = capability_interface_from_string(raw, path)
                        if spec_type == 'semantic_capability_interface':
                            iface = semantic_capability_interface_from_string(raw, path)
                        if spec_type in ['capability_interface', 'semantic_capability_interface']:
                            iface.name = '{package}/{name}'.format(package=package_name, name=iface.name)
                            if iface.name not in self.__default_providers:
                                default_provider = ''
                            else:
                                default_provider = self.__default_providers[iface.name]
                        if iface and iface.name == req.capability_spec:
                            response.capability_spec = CapabilitySpec(package_name, spec_type, raw, default_provider)
                            return response
        raise RuntimeError("Could not find requested spec '{0}'".format(req.capability_spec))

    def handle_start_capability(self, req):
        return self.__catch_and_log(self._handle_start_capability, req)

    def _handle_start_capability(self, req):
        msg = "Request to start capability '{0}'".format(req.capability)
        if req.preferred_provider:
            msg += " with provider '{0}'".format(req.preferred_provider)
        rospy.loginfo(msg)
        ret = self.__start_capability(req.capability, req.preferred_provider)
        return StartCapabilityResponse(ret or False)

    def handle_stop_capability(self, req):
        return self.__catch_and_log(self._handle_stop_capability, req)

    def _handle_stop_capability(self, req):
        rospy.loginfo("Request to stop capability '{0}'".format(req.capability))
        capability = req.capability
        if capability not in self.__capability_instances:
            raise RuntimeError("No Capability '{0}' running".format(capability))
        self.__stop_capability(capability)
        return StopCapabilityResponse(True)

    def handle_establish_bond(self, req):
        return self.__catch_and_log(self._handle_establish_bond, req)

    def _handle_establish_bond(self, req):
        rospy.loginfo("Request to establish a bond")
        bond_id = str(uuid.uuid1())

        def on_formed():
            rospy.loginfo("Bond formed with bond_id of '{0}'"
                          .format(bond_id))

        def on_broken():
            # if bond_id in self.__bonds:
            rospy.loginfo("Bond with bond id '{0}' was broken, freeing associated capabilities"
                          .format(bond_id))
            self.__free_capabilities_by_bond_id(bond_id)
            del self.__bonds[bond_id]

        self.__bonds[bond_id] = Bond(self.__bond_topic, bond_id, on_broken=on_broken, on_formed=on_formed)
        self.__bonds[bond_id].start()
        return EstablishBondResponse(bond_id)

    def __free_capabilities_by_bond_id(self, bond_id):
        if bond_id in self.__bonds:
            for capability in self.__capability_instances.values():
                if bond_id in capability.bonds:
                    del capability.bonds[bond_id]
                    if capability.reference_count == 0:
                        rospy.loginfo("Capability '{0}' being stopped because it has zero references"
                                      .format(capability.interface))
                        self.__stop_capability(capability.interface)

    def __free_capability(self, capability_name, bond_id):
        if capability_name not in self.__capability_instances:
            # If you update this exception's message, then update the corresponding code
            # in capabilities.client.CapabilitiesClient.free_capability()
            raise RuntimeError("Cannot free Capability '{0}', because it is not running".format(capability_name))
        capability = self.__capability_instances[capability_name]
        if bond_id not in capability.bonds:
            raise RuntimeError("Given bond_id '{0}' not associated with given capability '{1}'"
                               .format(bond_id, capability_name))
        if capability.bonds[bond_id] == 0:  # pragma: no cover
            # this is defensive, it should never happen
            raise RuntimeError("Cannot free capability '{0}' for bond_id '{1}', it already has a reference count of 0"
                               .format(capability_name, bond_id))
        capability.bonds[bond_id] -= 1
        if capability.reference_count == 0:
            rospy.loginfo("Capability '{0}' being stopped because it has zero references"
                          .format(capability.interface))
            self.__stop_capability(capability.interface)

    def handle_free_capability(self, req):
        return self.__catch_and_log(self._handle_free_capability, req)

    def _handle_free_capability(self, req):
        rospy.loginfo("Request to free usage of capability '{0}' (bond id '{1}')"
                      .format(req.capability, req.bond_id))
        self.__free_capability(req.capability, req.bond_id)
        return FreeCapabilityResponse()

    def handle_use_capability(self, req):
        return self.__catch_and_log(self._handle_use_capability, req)

    def _handle_use_capability(self, req):
        msg = "Request to use capability '{0}'".format(req.capability)
        if req.preferred_provider:
            msg += " with provider '{0}'".format(req.preferred_provider)
        rospy.loginfo(msg)
        # Make sure the bond_id is valid
        if req.bond_id not in self.__bonds:
            raise RuntimeError("Invalid bond_id given to ~use_capability: '{0}'".format(req.bond_id))
        # Start the capability if it is not already running
        if req.capability not in self.__capability_instances:
            # This will raise if it failes to start the capability
            self.__start_capability(req.capability, req.preferred_provider)
        assert req.capability in self.__capability_instances  # Should be true
        # Get a handle ont he capability
        capability = self.__capability_instances[req.capability]
        if req.preferred_provider and capability.name != req.preferred_provider:
            raise RuntimeError("Requested to use capability '{0}' with preferred provider '{1}', "
                               .format(capability.interface, req.preferred_provider) +
                               "but the capability is already running with provider '{0}'"
                               .format(capability.name))
        if req.bond_id not in capability.bonds:
            capability.bonds[req.bond_id] = 0
        capability.bonds[req.bond_id] += 1

        return UseCapabilityResponse()

    def handle_reload_request(self, req):
        return self.__catch_and_log(self._handle_reload_request, req)

    def _handle_reload_request(self, req):
        rospy.loginfo("Reloading capabilities...")
        self.__load_capabilities()
        return EmptyResponse()

    def handle_get_interfaces(self, req):
        return self.__catch_and_log(self._handle_get_interfaces, req)

    def _handle_get_interfaces(self, req):
        return GetInterfacesResponse(self.__spec_index.interface_names)

    def handle_get_providers(self, req):
        return self.__catch_and_log(self._handle_get_providers, req)

    def _handle_get_providers(self, req):
        if req.interface:
            if req.interface not in self.__spec_index.interfaces.keys() + self.__spec_index.semantic_interfaces.keys():
                raise RuntimeError("Capability Interface '{0}' not found.".format(req.interface))
            providers = self.__get_providers_for_interface(req.interface, allow_semantic=req.include_semantic).keys()
            default_provider = self.__default_providers[req.interface]
        else:
            providers = self.__spec_index.provider_names
            default_provider = ''
        return GetProvidersResponse(providers, default_provider)

    def handle_get_semantic_interfaces(self, req):
        return self.__catch_and_log(self._handle_get_semantic_interfaces, req)

    def _handle_get_semantic_interfaces(self, req):
        if req.interface:
            sifaces = [si.name
                       for si in self.__spec_index.semantic_interfaces.values()
                       if si.redefines == req.interface]
        else:
            sifaces = self.__spec_index.semantic_interface_names
        return GetSemanticInterfacesResponse(sifaces)

    def handle_get_running_capabilities(self, req):
        return self.__catch_and_log(self._handle_get_running_capabilities, req)

    def _handle_get_running_capabilities(self, req):
        resp = GetRunningCapabilitiesResponse()
        for instance in self.__capability_instances.values():
            if instance.state not in ['running']:  # pragma: no cover
                continue
            running_capability = RunningCapability()
            running_capability.capability = Capability(instance.interface, instance.name)
            running_capability.started_by = instance.started_by
            running_capability.pid = instance.pid
            rdepends = get_reverse_depends(instance.interface, self.__capability_instances.values())
            for dep in rdepends:
                running_capability.dependent_capabilities.append(Capability(dep.interface, dep.name))
            resp.running_capabilities.append(running_capability)
        return resp

    def handle_get_nodelet_manager_name(self, req):
        return self.__catch_and_log(self._handle_get_nodelet_manager_name, req)

    def _handle_get_nodelet_manager_name(self, req):
        resp = GetNodeletManagerNameResponse()
        resp.nodelet_manager_name = self.__launch_manager.nodelet_manager_name
        return resp

    def handle_get_remappings(self, req):
        return self.__catch_and_log(self._handle_get_remappings, req)

    def _handle_get_remappings(self, req):
        interface = None
        if req.spec in self.__capability_instances.keys():
            interface = self.__capability_instances[req.spec]
        else:
            providers = dict([(i.provider.name, i) for i in self.__capability_instances.values()])
            if req.spec not in providers:
                raise RuntimeError("Spec '{0}' is neither a running Interface nor a running Provider."
                                   .format(req.spec))
            interface = providers[req.spec]
        resp = GetRemappingsResponse()
        remappings = {
            'topics': {},
            'services': {},
            'actions': {},
            'parameters': {}
        }
        # Iterate this instance and its recursive dependencies
        for iface in reversed(self.__get_capability_instances_from_provider(interface.provider)):
            # For each iterate over their remappings and add them to the combined remappings,
            # flattening the remappings as you go
            for map_type, mapping in iface.provider.remappings_by_type.items():
                assert map_type in remappings
                remappings[map_type].update(mapping)
            # Collapse remapping chains
            for mapping in remappings.values():
                for key, value in mapping.items():
                    if value in mapping:
                        mapping[key] = mapping[value]
                        del mapping[value]
        for map_type, mapping in remappings.items():
            resp_mapping = getattr(resp, map_type)
            for key, value in mapping.items():
                remapping = Remapping()
                remapping.key = key
                remapping.value = value
                resp_mapping.append(remapping)
        return resp


def create_parser():
    parser = argparse.ArgumentParser(description="Runs the capability server")
    add = parser.add_argument
    add('package_path', nargs='?',
        help="Overrides ROS_PACKAGE_PATH when discovering capabilities")
    add('--screen', '-s', action='store_true', default=False,
        help="Passes `--screen` down to roslaunch, `use_screen` rosparam takes priority.")
    return parser


def main(sysargv=None):
    sys.argv = rospy.myargv(argv=sys.argv)

    parser = create_parser()
    args = parser.parse_args(sysargv)

    ros_package_path = args.package_path or os.getenv('ROS_PACKAGE_PATH', '')
    ros_package_path = [x for x in ros_package_path.split(':') if x]
    if not ros_package_path:
        sys.exit('No package paths specified, set ROS_PACKAGE_PATH or '
                 'pass them as an argument')
    # Extend the ROS_PACKAGE_PATH
    os.environ['ROS_PACKAGE_PATH'] = ':'.join(
        os.getenv('ROS_PACKAGE_PATH', '').split(':') + ros_package_path)

    rospy.init_node('capability_server')

    capability_server = CapabilityServer(ros_package_path, screen=args.screen)
    capability_server.spin()
    capability_server.shutdown()
