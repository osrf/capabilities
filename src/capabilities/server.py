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
import threading

import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from capabilities.srv import GetInterfaces
from capabilities.srv import GetInterfacesResponse
from capabilities.srv import GetProviders
from capabilities.srv import GetProvidersResponse
from capabilities.srv import GetSemanticInterfaces
from capabilities.srv import GetSemanticInterfacesResponse
from capabilities.srv import StartCapability
from capabilities.srv import StartCapabilityResponse
from capabilities.srv import StopCapability
from capabilities.srv import StopCapabilityResponse

from capabilities.discovery import package_index_from_package_path
from capabilities.discovery import spec_file_index_from_package_index
from capabilities.discovery import spec_index_from_spec_file_index
from capabilities.discovery import DuplicateNameException
from capabilities.discovery import InterfaceNameNotFoundException

from capabilities.launch_manager import LaunchManager

from capabilities.msg import CapabilityEvent


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
        if self.state != 'stopping':
            rospy.logerr(
                "Capability Instance '{0}' ".format(self.name) +
                "terminated unexpectedly, it was previously in the " +
                "'{0}' state.".format(self.state))
            return False
        self.__state = 'terminated'
        return True


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
    for instance in capability_instances.values():
        if name in instance.depends_on:
            rdepends.append(instance)
    return rdepends


class CapabilityServer(object):
    """A class to expose the :py:class:`discovery.SpecIndex` over a ROS API
    """

    def __init__(self, package_paths):
        self.__package_paths = package_paths
        self.__spec_index = None
        self.__graph_lock = threading.Lock()
        self.__capability_instances = {}
        self.__launch_manager = LaunchManager()

    def spin(self):
        """Starts the capability server by setting up ROS comms, then spins"""
        self.__load_capabilities()

        self.__start_capability_service = rospy.Service(
            'start_capability', StartCapability, self.handle_start_capability)

        self.__start_capability_service = rospy.Service(
            'stop_capability', StopCapability, self.handle_stop_capability)

        self.__reload_service = rospy.Service(
            'reload_capabilities', Empty, self.handle_reload_request)

        self.__interfaces_service = rospy.Service(
            'get_interfaces', GetInterfaces, self.handle_get_interfaces)

        self.__providers_service = rospy.Service(
            'get_providers', GetProviders, self.handle_get_providers)

        self.__semantic_interfaces_service = rospy.Service(
            'get_semantic_interfaces', GetSemanticInterfaces,
            self.handle_get_semantic_interfaces)

        rospy.Subscriber(
            'events', CapabilityEvent, self.handle_capability_events)

        rospy.loginfo("Capability Server Ready")

        rospy.spin()

    def shutdown(self):
        """Stops the capability server and cleans up any running processes"""
        self.__launch_manager.stop()

    def handle_capability_events(self, event):
        """Callback for handling messages (events) from the /events topic

        This callback only process events generated by this node.

        :param event: ROS message encapsulating an event
        :type event: :py:class:`capabilities.msgs.CapabilityEvent`
        """
        # Ignore any publications which we did not send (external publishers)
        if event._connection_header['callerid'] != rospy.get_name():
            return
        # Update the capability
        capability = event.capability
        with self.__graph_lock:
            if capability not in self.__capability_instances:
                rospy.logerr("Unknown capability instance: '{0}'"
                             .format(capability))
                return
            instance = self.__capability_instances[capability]
            if event.type == event.LAUNCHED:
                if instance.canceled:
                    self.__stop_capability(instance.name)
                else:
                    instance.launched(event.pid)
            elif event.type == event.TERMINATED:
                instance.terminated()
                rospy.loginfo(
                    "Capability Provider '{0}' for Capability '{1}' "
                    .format(event.provider, event.capability) +
                    "has terminated.")
                # Stop or cancel any dependents
                reverse_depends = get_reverse_depends(
                    instance, self.__capability_instances)
                for dependency in reverse_depends:
                    if dependency.state == 'running':
                        self.__stop_capability(dependency.name)
                    if dependency.state == 'waiting':
                        dependency.cancel()
            elif event.type == event.STOPPED:
                self.__capability_instances[capability].stopped()
            # Update the graph
            self.__update_graph()

    def __update_graph(self):
        # collect all of the waiting capabilities
        waiting = [x
                   for x in self.__capability_instances.values()
                   if x.state == 'waiting']
        # If any of the waiting have no blocking dependencies start them
        for instance in waiting:
            blocking_dependencies = []
            for dependency_name in instance.depends_on:
                if dependency_name not in self.__capability_instances:
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
        # collect all of the terminated capabilities
        terminated = [x
                      for x in self.__capability_instances.values()
                      if x.state == 'terminated']
        # Remove terminated instances
        for instance in terminated:
            del self.__capability_instances[instance.interface]

    def __stop_capability(self, name):
        if name not in self.__capability_instances:
            rospy.logerr("Inconsistent capability run graph, asked to stop " +
                         "capability '{0}', ".format(name) +
                         "which is not in the list of capability instances.")
            return
        self.__launch_manager.stop_capability_provider(
            self.__capability_instances[name].pid)

    def __get_capability_instances_from_provider(self, provider):
        def get_provider_dependencies(provider):
            result = []
            for interface, dep in provider.dependencies.items():
                provider_name = dep.provider
                if provider_name is None:
                    capability = dep.capability
                    providers = self.__get_providers_for_interface(capability)
                    provider_name = providers.keys()[0]
                if provider_name not in self.__spec_index.providers:
                    raise RuntimeError("Capability Provider '{0}' not found"
                                       .format(provider_name))
                dep_provider = self.__spec_index.providers[provider_name]
                result.append((dep_provider, provider.name))
            return result

        instances = []
        providers = [(provider, 'user service call')]
        while providers:
            curr, reason = providers.pop()
            providers.extend(get_provider_dependencies(curr))
            curr_path = self.__spec_index.provider_paths[curr.name]
            instances.append(CapabilityInstance(curr, curr_path, started_by=reason))
        return instances

    def __get_providers_for_interface(self, interface):
        providers = dict([(p.name, p)
                          for p in self.__spec_index.providers.values()
                          if p.implements == interface])
        if not providers:
            raise RuntimeError("No providers for Capability '{0}'"
                               .format(interface))
        return providers

    def __start_capability(self, capability, preferred_provider):
        if capability not in self.__spec_index.interfaces:
            raise RuntimeError("Capability '{0}' not found.".format(capability))
        providers = self.__get_providers_for_interface(capability)
        if preferred_provider:
            if preferred_provider not in providers:
                raise RuntimeError(
                    "Capability Provider '{0}' not found for Capability '{1}'"
                    .format(preferred_provider, capability))
            provider = providers[preferred_provider]
        else:
            provider = providers.values()[0]
        instances = self.__get_capability_instances_from_provider(provider)
        with self.__graph_lock:
            self.__capability_instances.update(dict([(x.interface, x) for x in instances]))
            self.__update_graph()
        return True

    def __load_capabilities(self):
        package_index = package_index_from_package_path(self.__package_paths)
        spec_file_index = spec_file_index_from_package_index(package_index)
        spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
        if errors:
            rospy.logerr("Errors were encountered loading capabilities:")
            for error in errors:
                if type(error) == DuplicateNameException:
                    rospy.logerr("  DuplicateNameException: " + str(error))
                elif type(error) == InterfaceNameNotFoundException:
                    rospy.logerr("  InterfaceNameNotFoundException: " +
                                 str(error))
                else:
                    rospy.logerr("  " + str(type(error)) + ": " + str(error))
        self.__spec_index = spec_index

    def handle_start_capability(self, req):
        msg = "Request to start capability '{0}'".format(req.capability)
        if req.preferred_provider:
            msg += " with provider '{0}'".format(req.preferred_provider)
        rospy.loginfo(msg)
        ret = self.__start_capability(req.capability, req.preferred_provider)
        return StartCapabilityResponse(ret or False)

    def handle_stop_capability(self, req):
        rospy.loginfo("Request to stop capability '{0}'".format(req.capability))
        capability = req.capability
        if capability not in self.__capability_instances:
            raise RuntimeError("No Capability '{0}' running".format(capability))
        self.__capability_instances[capability].stopped()
        self.__stop_capability(req.capability)
        return StopCapabilityResponse(True)

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

    capability_server = CapabilityServer(ros_package_path)
    capability_server.spin()
    capability_server.shutdown()
