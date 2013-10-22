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
This module implements the Capability Interface concept

This module contains free functions which serve as factories for the :py:class:`CapabilityInterface` class.
These factories can take the spec file for a Capability Interface and create a
:py:class:`CapabilityInterface` instance out of it.
The :py:class:`Interface` class encapsulates the interface itself, representing the topics, services, params, etc...
The :py:class:`CapabilityInterface` class encapsulates meta information about the Interface like the name,
description, and extends the :py:class:`Interface` class.
The :py:class:`Interface` class is made up of :py:class:`InterfaceElement`'s which represent
the elements of a Capability Interface, each consisting of a name, type, and description.

With an Interface RGBCamera.yaml, like this::

    %YAML 1.1
    # Capability Interface file for RGBCamera
    ---
    # Name of the Interface
    name: RGBCamera
    spec_version: 1
    spec_type: interface
    description: 'This Capability describes the interface exposed by a generic RGBCamera.'
    interface:
      # Topics
      topics:
        # Topics provided by the interface
        provides:
          'camera/image_raw':
            type: 'sensor_msgs/Image'
            description: 'raw image from the camera'
          'camera/camera_info':
            type: 'sensor_msgs/CameraInfo'
        # Topics expect by the interface
        requires: {}
      # Services provided by the interface
      services:
        'camera/set_info':
          type: 'sensor_msgs/SetCameraInfo'

You can use this interface like so::

    >>> from pprint import pprint
    >>> from capabilities.specs.interface import capability_interface_from_file_path
    >>> ci = capability_interface_from_file_path('test/unit/specs/interfaces/RGBCamera.yaml')
    >>> pprint(ci.topics)
    {'camera/camera_info': <capabilities.specs.interface.InterfaceElement object at 0x10736ced0>,
     'camera/image_raw': <capabilities.specs.interface.InterfaceElement object at 0x10736cf10>}
    >>> print(ci.topics.values()[0].name)
    camera/image_raw
    >>> print(ci.topics.values()[0].type)
    sensor_msgs/Image
    >>> print(ci.topics.values()[0].description)
    raw image from the camera
    >>> pprint(ci.required_services)
    {}
    >>> pprint(ci.provided_topics)
    {'camera/camera_info': <capabilities.specs.interface.InterfaceElement object at 0x10774ce10>,
     'camera/image_raw': <capabilities.specs.interface.InterfaceElement object at 0x10774ce50>}
    >>> pprint(ci.actions)
    {}

"""

from __future__ import print_function

import difflib
import os
import yaml


class InvalidInterface(Exception):
    """InvalidInterface exception"""
    def __init__(self, msg, file_name):
        self.file_name = file_name
        Exception.__init__(self, "In interface spec file '{0}': {1}".format(file_name, msg))


def capability_interface_from_file_path(file_path):
    """Creates a CapabilityInterface instance from a spec file at a given path

    See :py:func:`capability_interface_from_dict` for list of possible exceptions

    :param file_path: location of the Capability Interface spec file
    :type file_path: str
    :returns: CapabilityInterface instance, populated with data from the spec file
    :rtype: :py:class:`CapabilityInterface`
    :raises: :py:exc:`OSError` if the given file does not exist
    """
    with open(os.path.abspath(file_path), 'r') as f:
        return capability_interface_from_dict(yaml.load(f), file_path)


def capability_interface_from_file(file_handle):
    """Creates a CapabilityInterface instance from a given spec file handle

    See :py:func:`capability_interface_from_dict` for list of possible exceptions

    :param file_handle: file handle for the Capability Interface spec file
    :type file_handle: file
    :returns: CapabilityInterface instance, populated with data from the spec file
    :rtype: :py:class:`CapabilityInterface`
    :raises: :py:exc:`OSError` if the given file does not exist
    """
    return capability_interface_from_dict(yaml.load(file_handle.read()), file_handle.name)


def capability_interface_from_string(string, file_name='<string>'):
    """Creates a CapabilityInterface instance from a string containing the spec

    See :py:func:`capability_interface_from_dict` for list of possible exceptions

    :param string: Capability Interface spec
    :type string: str
    :param file_name: Name of the file where this spec originated (defaults to '<string>')
    :param file_name: str
    :returns: CapabilityInterface instance, populated with the provided spec
    :rtype: :py:class:`CapabilityInterface`
    :raises: :py:exc:`AttributeError` if the given value for string is not a str
    """
    return capability_interface_from_dict(yaml.load(string), file_name)


def capability_interface_from_dict(spec, file_name='<dict>'):
    """Creates a CapabilityInterface instance from a dict version of the spec

    :param string: Capability Interface spec
    :type string: dict
    :param file_name: Name of the file where this spec originated (defaults to '<dict>')
    :param file_name: str
    :returns: CapabilityInterface instance, populated with the provided spec
    :rtype: :py:class:`CapabilityInterface`
    :raises: :py:exc:`InvalidInterface` if the spec is not complete or has invalid entries
    """
    if 'name' not in spec:
        raise InvalidInterface('No name specified', file_name)
    name = spec['name']
    if 'spec_type' not in spec:
        raise InvalidInterface('No spec type specified', file_name)
    if spec['spec_type'] != 'interface':
        raise InvalidInterface("Invalid spec type, expected 'interface' got: '{0}'".format(spec['spec_type']),
                               file_name)
    if 'spec_version' not in spec:
        raise InvalidInterface('No spec version specified', file_name)
    spec_version = int(spec['spec_version'])
    if spec_version != 1:
        raise InvalidInterface("Invalid spec version: '{0}'".format(spec_version), file_name)
    description = spec.get('description', 'No description given.')
    capability_interface = CapabilityInterface(name, spec_version, description)
    interface = spec.get('interface', {})
    for key in interface:
        valid_interface_keys = ['topics', 'services', 'actions', 'parameters', 'dynamic_parameters']
        if key in valid_interface_keys[:-1]:
            element_groups = interface[key]
            if not isinstance(element_groups, dict):
                raise InvalidInterface("Invalid {0} section, expected dict got: '{0}'".format(type(element_groups)),
                                       file_name)
            __process_interface_element(key[:-1], capability_interface, element_groups, file_name)
        elif key == 'dynamic_parameters':
            dynamic_parameters = interface[key]
            if not isinstance(dynamic_parameters, list):
                raise InvalidInterface("Invalid dynamic_parameters entry, expect list got: '{0}'"
                                       .format(type(dynamic_parameters)), file_name)
            for dynamic_parameter in dynamic_parameters:
                try:
                    capability_interface.add_dynamic_parameter(dynamic_parameter)
                except (ValueError, RuntimeError) as e:
                    raise InvalidInterface(str(e), file_name)
        else:
            matches = difflib.get_close_matches(key, valid_interface_keys)
            msg = ""
            if len(matches) > 1:
                msg += ", did you mean: '{0}{1}'?".format("', '".join(matches[:-1]), "', or '" + matches[-1])
            elif matches:
                msg += ", did you mean: '{0}'?".format(matches[0])
            raise InvalidInterface("Invalid interface section: '{0}'".format(key) + msg, file_name)
    return capability_interface


def __process_interface_element(element_type, capability_interface, element_groups, file_name):
    def __add_element_to_interface(name, element, group_name=None):
        if 'type' not in element:
            raise InvalidInterface("{0} has no type: '{1}'".format(element_type.capitalize(), name), file_name)
        description = element.get('description', None)
        interface_element = InterfaceElement(name, element_type, element['type'], description)
        add = getattr(capability_interface, 'add_' + element_type)
        try:
            add(name, interface_element, group_name)
        except (ValueError, RuntimeError) as e:
            raise InvalidInterface(str(e), file_name)

    for group in element_groups:
        if group in ['requires', 'provides']:
            elements = element_groups[group] or {}
            for name, element in elements.iteritems():
                __add_element_to_interface(name, element, group)
        else:
            __add_element_to_interface(group, element_groups[group])


class Interface(object):
    """Represents the Interface part of a Capability Interface

    An Interface can consist of zero to all of these elements:

    - topics: (name/type)

      - requires: Expected topics
      - provides: Provided topics

    - services: (name/type)

      - requires: Expected services
      - provides: Provided services

    - actions: (name/type)

      - requires: Expected actions
      - provides: Provided actions

    - parameters: (name/type/description)

      - requires: Parameters which should be set for the interface, e.g. map_resolution
      - provides: Parameters which the interface provides, e.g. robot_model

    - dynamic_parameters: List of dynamic_parameters which are available (name)

    The groupings 'requires' and 'provides' are purely semantic,
    and are not used by the capability system programatically,
    but instead are there to give more meaning to the developers.
    """
    def __init__(self):
        self.__topics = {}
        self.__services = {}
        self.__actions = {}
        self.__parameters = {}
        self.__dynamic_parameters = []
        self.__element_containers = {
            'topic': self.__topics,
            'service': self.__services,
            'action': self.__actions,
            'parameter': self.__parameters,
            'dynamic_parameter': self.__dynamic_parameters,
        }

    def __check_element(self, element):
        if not isinstance(element, InterfaceElement):
            raise TypeError("Invalid element, expected InterfaceElement got: '{0}'".format(type(element)))

    def __check_name(self, name):
        if not isinstance(name, str):
            raise TypeError("Invalid element name, expected str got: '{0}'".format(type(name)))

    def __add_element(self, kind, name, element, grouping):
        # Make sure this is a valid kind of element
        if kind not in self.__element_containers:
            raise ValueError("Invalid element kind: '{0}'".format(kind))
        # Make sure the name is a str
        self.__check_name(name)
        # Make sure this is actually an InterfaceElement
        self.__check_element(element)
        # Make sure the grouping is valid
        grouping = grouping or 'no group'
        if grouping not in ['no group', 'requires', 'provides']:
            raise ValueError("Invalid grouping: '{0}'".format(grouping))
        container = self.__element_containers[kind]
        # Create the group in the container if needed
        if grouping not in container:
            container[grouping] = {}
        # Store the element in the container by group and name
        container[grouping][name] = element

    @property
    def topics(self):
        return dict([(name, topics[name]) for topics in self.__topics.values() for name in topics])

    @property
    def required_topics(self):
        return dict(self.__topics.get('requires', {}))

    @property
    def provided_topics(self):
        return dict(self.__topics.get('provides', {}))

    def add_topic(self, topic_name, interface_element, grouping=None):
        """Adds a topic to the Interface

        :param topic_name: name of the topic
        :type topic_name: str
        :param interface_element: :py:class:`InterfaceElement` instance for the topic
        :type interface_element: :py:class:`InterfaceElement`
        :param grouping: semantic grouping of the topic, 'requires', 'provides', or None
        :type grouping: :py:obj:`str` or :py:obj:`None`
        :raises: :py:exc:`ValueError` if the grouping is invalid
        :raises: :py:exc:`RuntimeError` if the topic is already in the interface
        """
        if topic_name in self.topics:
            raise RuntimeError("Interface has topic listed twice: '{0}'".format(topic_name))
        self.__add_element('topic', topic_name, interface_element, grouping)

    @property
    def services(self):
        return dict([(name, services[name]) for services in self.__services.values() for name in services])

    @property
    def required_services(self):
        return dict(self.__services.get('requires', {}))

    @property
    def provided_services(self):
        return dict(self.__services.get('provides', {}))

    def add_service(self, service_name, interface_element, grouping=None):
        """Adds a service to the Interface

        :param service_name: name of the service
        :type service_name: str
        :param interface_element: :py:class:`InterfaceElement` instance for the service
        :type interface_element: :py:class:`InterfaceElement`
        :param grouping: semantic grouping of the service, 'requires', 'provides', or None
        :type grouping: :py:obj:`str` or :py:obj:`None`
        :raises: :py:exc:`ValueError` if the grouping is invalid
        :raises: :py:exc:`RuntimeError` if the service is already in the interface
        """
        if service_name in self.services:
            raise RuntimeError("Interface has service listed twice: '{0}'".format(service_name))
        self.__add_element('service', service_name, interface_element, grouping)

    @property
    def actions(self):
        return dict([(name, actions[name]) for actions in self.__actions.values() for name in actions])

    @property
    def required_actions(self):
        return dict(self.__actions.get('requires', {}))

    @property
    def provided_actions(self):
        return dict(self.__actions.get('provides', {}))

    def add_action(self, action_name, interface_element, grouping=None):
        """Adds a action to the Interface

        :param action_name: name of the action
        :type action_name: str
        :param interface_element: :py:class:`InterfaceElement` instance for the action
        :type interface_element: :py:class:`InterfaceElement`
        :param grouping: semantic grouping of the action, 'requires', 'provides', or None
        :type grouping: :py:obj:`str` or :py:obj:`None`
        :raises: :py:exc:`ValueError` if the grouping is invalid
        :raises: :py:exc:`RuntimeError` if the action is already in the interface
        """
        if action_name in self.actions:
            raise RuntimeError("Interface has action listed twice: '{0}'".format(action_name))
        self.__add_element('action', action_name, interface_element, grouping)

    @property
    def parameters(self):
        return dict([(name, actions[name]) for actions in self.__parameters.values() for name in actions])

    @property
    def required_parameters(self):
        return dict(self.__parameters.get('requires', {}))

    @property
    def provided_parameters(self):
        return dict(self.__parameters.get('provides', {}))

    def add_parameter(self, parameter_name, interface_element, grouping=None):
        """Adds a parameter to the Interface

        :param parameter_name: name of the parameter
        :type parameter_name: str
        :param interface_element: :py:class:`InterfaceElement` instance for the parameter
        :type interface_element: :py:class:`InterfaceElement`
        :param grouping: semantic grouping of the parameter, 'requires', 'provides', or None
        :type grouping: :py:obj:`str` or :py:obj:`None`
        :raises: :py:exc:`ValueError` if the grouping is invalid
        :raises: :py:exc:`RuntimeError` if the parameter is already in the interface
        """
        if parameter_name in self.parameters:
            raise RuntimeError("Interface has parameter listed twice: '{0}'".format(parameter_name))
        self.__add_element('parameter', parameter_name, interface_element, grouping)

    @property
    def dynamic_parameters(self):
        return list(self.__dynamic_parameters)

    def add_dynamic_parameter(self, dynamic_parameter_name):
        """Adds a dynamic_parameter to the Interface

        :param dynamic_parameter_name: name of the dynamic_parameter
        :type dynamic_parameter_name: str
        :raises: :py:exc:`RuntimeError` if the dynamic_parameter is already in the interface
        """
        if dynamic_parameter_name in self.dynamic_parameters:
            raise RuntimeError("Interface has dynamic parameter listed twice: '{0}'".format(dynamic_parameter_name))
        self.__dynamic_parameters.append(dynamic_parameter_name)


class InterfaceElement(object):
    """Represents a part of an interface

    An interface element is defined by:

    - name: topic, service, action, or parameter name
    - element kind: topic, service, action, parameter, or dynamic_parameter
    - type: topic, service or action type (pkg/msg) or parameter type
    - description: free form description of the element

    :raises: :py:exc:`ValueError` if the element kind is invalid
    """
    valid_element_kinds = ['topic', 'service', 'action', 'parameter', 'dynamic_parameter']

    def __init__(self, name, element_kind, element_type=None, description=None):
        self.name = name
        if element_kind not in self.valid_element_kinds:
            raise ValueError("Invalid element kind: '{0}'".format(element_kind))
        self.kind = element_kind
        self.type = self.element_type = element_type
        self.description = description

    def __str__(self):
        return """name: {name}
kind: {kind}
element type: {element_type}
description:
  {description}""".format(**self.__dict__)


class CapabilityInterface(Interface):
    """Represents a Capability Interface

    A Capability Interface is defined by:

    - name (str): name of the interface
    - spec_type (str): type of the interface specification (has to be 'interface')
    - spec_version (int): version of the interface specification
    - description (str): free form description of the interface
    - default_provider (str): name of the default provider for this interface, defaults to 'unknown'
    - interface (:py:class:`Interface`): representation of components which make up the interface
    """
    spec_type = 'interface'

    def __init__(self, name, spec_version, description=None):
        self.name = name
        self.spec_version = spec_version
        self.description = description
        self.default_provider = 'unknown'
        Interface.__init__(self)

    def __str__(self):
        elements = "topics:\n"
        required_and_provided = list(self.required_topics.keys() + self.provided_topics.keys())
        both = [x for x in self.topics if x not in required_and_provided]
        for name, topic in self.topics.items():
            if name not in both:
                continue
            elements += "      '" + name + "':\n        "
            elements += "\n        ".join(str(topic).splitlines())
            elements += "\n"
        elements += "      requires:\n"
        for name, topic in self.required_topics.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(topic).splitlines())
            elements += "\n"
        elements += "      provides:\n"
        for name, topic in self.provided_topics.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(topic).splitlines())
            elements += "\n"
        elements += "    services:\n"
        required_and_provided = list(self.required_services.keys() + self.provided_services.keys())
        both = [x for x in self.services if x not in required_and_provided]
        for name, service in self.services.items():
            if name not in both:
                continue
            elements += "      '" + name + "':\n        "
            elements += "\n        ".join(str(service).splitlines())
            elements += "\n"
        elements += "      requires:\n"
        for name, service in self.required_services.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(service).splitlines())
            elements += "\n"
        elements += "      provides:\n"
        for name, service in self.provided_services.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(service).splitlines())
            elements += "\n"
        elements += "    actions:\n"
        required_and_provided = list(self.required_actions.keys() + self.provided_actions.keys())
        both = [x for x in self.actions if x not in required_and_provided]
        for name, action in self.actions.items():
            if name not in both:
                continue
            elements += "      '" + name + "':\n        "
            elements += "\n        ".join(str(action).splitlines())
            elements += "\n"
        elements += "      requires:\n"
        for name, action in self.required_actions.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(action).splitlines())
            elements += "\n"
        elements += "      provides:\n"
        for name, action in self.provided_actions.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(action).splitlines())
            elements += "\n"
        elements += "    parameters:\n"
        required_and_provided = list(self.required_parameters.keys() + self.provided_parameters.keys())
        both = [x for x in self.parameters if x not in required_and_provided]
        for name, parameter in self.parameters.items():
            if name not in both:
                continue
            elements += "      '" + name + "':\n        "
            elements += "\n        ".join(str(parameter).splitlines())
            elements += "\n"
        elements += "      requires:\n"
        for name, parameter in self.required_parameters.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(parameter).splitlines())
            elements += "\n"
        elements += "      provides:\n"
        for name, parameter in self.provided_parameters.items():
            elements += "        '" + name + "':\n        "
            elements += "\n          ".join(str(parameter).splitlines())
            elements += "\n"
        elements += "    dynamic parameters:\n"
        for parameter in self.dynamic_parameters:
            elements += "\n      " + str(parameter)
            elements += "\n"
        return """Capability Interface:
{{
  name: {name}
  spec version: {spec_version}
  default provider: {default_provider}
  description:
    {description}
  elements:
    {elements}
}}""".format(elements=elements, **self.__dict__)
