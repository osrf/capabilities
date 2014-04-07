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
This module implements the Capability Provider concept

This module contains free functions which serve as factories for the :py:class:`CapabilityProvider` class.
These factories can take the spec file for a Capability Provider and create a
:py:class:`CapabilityProvider` instance out of it.
The :py:class:`CapabilityProvider` class is designed to encapsualte the meta data
which describes the Capability Provider.

With a provider spec like this::

    %YAML 1.1
    ---
    name: navigation_nav_stack
    spec_version: 1
    spec_type: provider
    description: 'Implements the ability to navigate.'
    implements: navigation/Navigation
    launch_file: 'launch/navigation_nav_stack.launch'
    depends_on:
        'laser_capability/LaserObservation':
            provider: 'hokuyo_capability/hokuyo_base'
    remappings:
        topics:
            'scan': 'nav_stack/scan'
    nodelet_manager: 'nav_manager'

You can use this API like this::

    >>> from pprint import pprint
    >>> from capabilities.specs.provider import capability_provider_from_file_path
    >>> cp = capability_provider_from_file_path('test/unit/specs/providers/navigation_nav_stack.yaml')
    >>> pprint(cp.dependencies)
    {<capabilities.specs.common.SpecName object at 0x1099ebfd0>:
        <capabilities.specs.provider.DependsOnRelationship object at 0x109a3fa50>}
    >>> print(cp.dependencies['laser_capability/LaserObservation'])
    laser_capability/LaserObservation:
        preferred provider: hokuyo_capability/hokuyo_base
    >>> print(cp.launch_file)
    launch/navigation_nav_stack.launch
    >>> print(cp.implements)
    navigation/Navigation
    >>> print(cp.nodelet_manager)
    nav_manager

"""

from __future__ import print_function

import os
import yaml

from capabilities.specs.common import validate_spec_name

from capabilities.specs.remappings import RemapCollection


class InvalidProvider(Exception):
    """InvalidProvider exception"""
    def __init__(self, msg, file_name):
        self.file_name = file_name
        Exception.__init__(self, "In provider spec file '{0}': {1}".format(file_name, msg))


def capability_provider_from_file_path(file_path):
    """Creates a CapabilityProvider instance from a spec file at a given path

    See :py:func:`capability_provider_from_dict` for list of possible exceptions

    :param file_path: location of the Capability Provider spec file
    :type file_path: str
    :returns: CapabilityProvider instance, populated with data from the spec file
    :rtype: :py:class:`CapabilityProvider`
    :raises: :py:exc:`OSError` if the given file does not exist
    """
    with open(os.path.abspath(file_path), 'r') as f:
        return capability_provider_from_dict(yaml.load(f.read()), file_path)


def capability_provider_from_file(file_handle):
    """Creates a CapabilityProvider instance from a given spec file handle

    See :py:func:`capability_provider_from_dict` for list of possible exceptions

    :param file_handle: file handle for the Capability Provider spec file
    :type file_handle: file
    :returns: CapabilityProvider instance, populated with data from the spec file
    :rtype: :py:class:`CapabilityProvider`
    :raises: :py:exc:`OSError` if the given file does not exist
    """
    return capability_provider_from_dict(yaml.load(file_handle.read()), file_handle.name)


def capability_provider_from_string(string, file_name='<string>'):
    """Creates a CapabilityProvider instance from a string containing the spec

    See :py:func:`capability_provider_from_dict` for list of possible exceptions

    :param string: Capability Provider spec
    :type string: str
    :param file_name: Name of the file where this spec originated (defaults to '<string>')
    :type file_name: str
    :returns: CapabilityProvider instance, populated with the provided spec
    :rtype: :py:class:`CapabilityProvider`
    :raises: :py:exc:`AttributeError` if the given value for string is not a str
    """
    return capability_provider_from_dict(yaml.load(string), file_name)


def capability_provider_from_dict(spec, file_name='<dict>'):
    """Creates a CapabilityProvider instance from a dict version of the spec

    :param string: Capability Provider spec
    :type string: dict
    :param file_name: Name of the file where this spec originated (defaults to '<dict>')
    :type file_name: str
    :returns: CapabilityProvider instance, populated with the provided spec
    :rtype: :py:class:`CapabilityProvider`
    :raises: :py:exc:`InvalidProvider` if the spec is not complete or has invalid entries
    """
    if 'name' not in spec:
        raise InvalidProvider('No name specified', file_name)
    name = spec['name']
    if 'spec_type' not in spec:
        raise InvalidProvider('No spec type specified', file_name)
    if spec['spec_type'] != 'provider':
        raise InvalidProvider("Invalid spec type, expected 'provider' got: '{0}'".format(spec['spec_type']),
                              file_name)
    if 'spec_version' not in spec:
        raise InvalidProvider('No spec version specified', file_name)
    spec_version = int(spec['spec_version'])
    if spec_version != 1:
        raise InvalidProvider("Invalid spec version: '{0}'".format(spec_version), file_name)
    if 'implements' not in spec:
        raise InvalidProvider("No implements specified", file_name)
    implements = spec['implements']
    try:
        validate_spec_name(implements)
    except (ValueError, AssertionError) as exc:
        raise InvalidProvider("Invalid spec name for implements: " + str(exc), file_name)
    launch_file = spec.get('launch_file', None)
    description = spec.get('description', 'No description given.')
    remappings = spec.get('remappings', {})
    if not isinstance(remappings, dict):
        raise InvalidProvider("Invalid remappings section, expected dict got: '{0}'".format(type(remappings)),
                              file_name)
    nodelet_manager = spec.get('nodelet_manager', None)
    if not isinstance(nodelet_manager, (str, type(None))):
        raise InvalidProvider("Invalid nodelet_manager, expected string got: '{0}'".format(type(nodelet_manager)),
                              file_name)
    try:
        capability_provider = CapabilityProvider(name, spec_version, implements, launch_file,
                                                 description, remappings, nodelet_manager)
    except (AssertionError, ValueError) as e:  # Catch remapping errors
            raise InvalidProvider(str(e), file_name)
    depends_on = spec.get('depends_on', {})
    if isinstance(depends_on, str):
        depends_on = [depends_on]
    if isinstance(depends_on, (list, tuple)):
        depends_on = dict([(x, {}) for x in depends_on])
    if not isinstance(depends_on, dict):
        raise InvalidProvider("Invalid depends_on section, expected dict got: '{0}'".format(type(depends_on)),
                              file_name)
    valid_conditionals = ['provider']
    for interface, conditions in depends_on.iteritems():
        if not isinstance(conditions, dict):
            raise InvalidProvider("Invalid depends_on conditional section, expected dict got: '{0}'"
                                  .format(type(conditions)), file_name)
        for key in conditions:
            if key not in valid_conditionals:
                raise InvalidProvider("Invalid depends_on interface condition '{0}', should be one of: '{1}'"
                                      .format(key, "', '".join(valid_conditionals)), file_name)
        preferred_provider = conditions.get('provider', None)
        capability_provider.add_depends_on(interface, preferred_provider)
    return capability_provider


class CapabilityProvider(object):
    """Represents a Capability Provider

    A Capability Provider is defined by:

    - name (str): name of the provider
    - spec_type (str): type of the specification (has to be 'provider')
    - spec_version (int): version of the provider specification
    - description (str): free form description of the provider
    - implements (str): Name of a Capability Interface which this provider implements
    - launch_file (str or None): Path to a launch file which runs the provider, None indicates no launch file to run
    - depends_on (dict): list of depends on relationships to Capabilities with remappings and provider preference
    - remappings (dict): map of ROS Names defined in the Capability to their new names for this provider
    - nodelet_manager (str or None): name of the nodelet manager used by the provider, this is an implementation hint
    """
    spec_type = 'provider'

    def __init__(self, name, spec_version, implements, launch_file=None,
                 description=None, remappings=None, nodelet_manager=None):
        self.name = name
        self.spec_version = spec_version
        self.description = description
        self.implements = implements
        self.launch_file = launch_file
        self.nodelet_manager = nodelet_manager
        self.__remap_collection = RemapCollection()
        self.add_remappings_by_dict(remappings or {})
        self.__depends_on = {}

    @property
    def remappings(self):
        return self.__remap_collection.remappings

    @property
    def remappings_by_type(self):
        return self.__remap_collection.remappings_by_type

    def add_remappings_by_dict(self, remappings_dict):
        self.__remap_collection.add_remappings_by_dict(remappings_dict)

    @property
    def dependencies(self):
        return self.__depends_on

    def depends_on(self, interface_name):
        return interface_name in self.__depends_on

    def add_depends_on(self, interface_name, preferred_provider=None):
        relationship = DependsOnRelationship(interface_name, preferred_provider)
        # The dict strucutre of YAML should prevent duplicate interface_name keys
        self.__depends_on[interface_name] = relationship

    def __str__(self):
        return """Capability Provider:
{{
  name: {name}
  spec version: {spec_version}
  implements: {implements}{nodelet_manager_str}
  description:
    {description}
{remappings_str}
  depend on relationships:
{depends_on_str}
}}
""".format(depends_on_str="[\n" + "\n".join([str(v) for v in self.__depends_on.values()]) + "\n]",
           remappings_str=str(self.__remap_collection) + "\n",
           nodelet_manager_str='' if self.nodelet_manager is None else '\n' + self.nodelet_manager + '\n',
           **self.__dict__)


class DependsOnRelationship(object):
    """Models the depends_on relationship between a Capability Provider and a Capability

    This relationship consists of:

    - capability_name (str): name of the Capability which is depended on
    - provider_preference (str): (optional) name of preferred provider for the Capability which is depended on
    """
    valid_remapping_types = ['topics', 'services', 'parameters', 'actions']

    def __init__(self, capability_name, preferred_provider):
        validate_spec_name(capability_name)
        if preferred_provider is not None:
            validate_spec_name(preferred_provider)
        self.name = capability_name
        self.capability_name = capability_name
        self.provider = preferred_provider
        self.preferred_provider = preferred_provider

    def __str__(self):
        msg = "{0}".format(self.name)
        if self.provider:
            msg += ":\n  preferred provider: {0}".format(self.provider)
        return msg
