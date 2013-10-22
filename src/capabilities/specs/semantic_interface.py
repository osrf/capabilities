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
This module implements the Semantic Capability Interface concept

This module contains free functions which serve as factories for the :py:class:`SemanticCapabilityInterface` class.
These factories can take the spec file for a Semantic Capability Interface and create a
:py:class:`SemanticCapabilityInterface` instance out of it.
The :py:class:`SemanticCapabilityInterface` class encapsulates meta information about the semantic interface.
The :py:class:`SemanticCapabilityInterface` class defines of:

- A capability which is being redefined, by name and remappings
- A name
- A global namespace (optional)
- Any ROS Name remappings (optional)

With an Semantic Capability Interface FrontRGBCamera.yaml, like this::

    %YAML 1.1
    ---
    name: FrontRGBCamera
    spec_version: 1
    spec_type: semantic_interface
    description: 'This is semantically the Front RGB camera.'
    redefines: 'a_package/RGBCamera'
    # First the global_namespace is applied to all ROS Names
    global_namespace: 'front_camera'
    # Then individual remappings are done, and override the global_namespace
    remappings:
      topics:
        'camera/image_raw': 'front_camera/image_raw'
        'camera/camera_info': 'front_camera/camera_info'
      services:
        'camera/set_info': 'front_camera/set_info'

You can use this interface like so::

    >>> from pprint import pprint
    >>> from capabilities.specs.semantic_interface import semantic_capability_interface_from_file_path
    >>> sci = semantic_capability_interface_from_file_path('test/specs/semantic_interfaces/FrontRGBCamera.yaml')
    >>> print(sci.redefines)
    a_package/RGBCamera
    >>> print(sci.global_namespace)
    front_camera
    >>> pprint(sci.remappings)
    {'camera/camera_info': 'front_camera/camera_info',
     'camera/image_raw': 'front_camera/image_raw',
     'camera/set_info': 'front_camera/set_info'}

"""

from __future__ import print_function

import os
import yaml

from capabilities.specs.common import validate_spec_name

from capabilities.specs.remappings import RemapCollection

try:
    __basestring = basestring
except NameError:  # pragma: no cover
    __basestring = str


class InvalidSemanticInterface(Exception):
    """InvalidSemanticInterface exception"""
    def __init__(self, msg, file_name):
        self.file_name = file_name
        Exception.__init__(self, "In semantic interface spec file '{0}': {1}".format(file_name, msg))


def semantic_capability_interface_from_file_path(file_path):
    """Creates a SemanticCapabilityInterface instance from a spec file at a given path

    See :py:func:`semantic_capability_interface_from_dict` for list of possible exceptions

    :param file_path: location of the Semantic Capability Interface spec file
    :type file_path: str
    :returns: SemanticCapabilityInterface instance, populated with data from the spec file
    :rtype: :py:class:`SemanticCapabilityInterface`
    :raises: :py:exc:`OSError` if the given file does not exist
    """
    with open(os.path.abspath(file_path), 'r') as f:
        return semantic_capability_interface_from_dict(yaml.load(f.read()), file_path)


def semantic_capability_interface_from_file(file_handle):
    """Creates a SemanticCapabilityInterface instance from a given spec file handle

    See :py:func:`semantic_capability_interface_from_dict` for list of possible exceptions

    :param file_handle: file handle for the Semantic Capability Interface spec file
    :type file_handle: file
    :returns: SemanticCapabilityInterface instance, populated with data from the spec file
    :rtype: :py:class:`SemanticCapabilityInterface`
    :raises: :py:exc:`OSError` if the given file does not exist
    """
    return semantic_capability_interface_from_dict(yaml.load(file_handle.read()), file_handle.name)


def semantic_capability_interface_from_string(string, file_name='<string>'):
    """Creates a SemanticCapabilityInterface instance from a string containing the spec

    See :py:func:`semantic_capability_interface_from_dict` for list of possible exceptions

    :param string: Semantic Capability Interface spec
    :type string: str
    :param file_name: Name of the file where this spec originated (defaults to '<string>')
    :type file_name: str
    :returns: SemanticCapabilityInterface instance, populated with the provided spec
    :rtype: :py:class:`SemanticCapabilityInterface`
    :raises: :py:exc:`AttributeError` if the given value for string is not a str
    """
    return semantic_capability_interface_from_dict(yaml.load(string), file_name)


def semantic_capability_interface_from_dict(spec, file_name='<dict>'):
    """Creates a SemanticCapabilityInterface instance from a dict version of the spec

    :param string: Semantic Capability Interface spec
    :type string: dict
    :param file_name: Name of the file where this spec originated (defaults to '<dict>')
    :type file_name: str
    :returns: SemanticCapabilityInterface instance, populated with the provided spec
    :rtype: :py:class:`SemanticCapabilityInterface`
    :raises: :py:exc:`InvalidSemanticInterface` if the spec is not complete or has invalid entries
    """
    if 'spec_type' not in spec:
        raise InvalidSemanticInterface('No spec type specified', file_name)
    if spec['spec_type'] != 'semantic_interface':
        raise InvalidSemanticInterface("Invalid spec type, expected 'semantic_interface' got: '{0}'"
                                       .format(spec['spec_type']), file_name)
    if 'spec_version' not in spec:
        raise InvalidSemanticInterface('No spec version specified', file_name)
    spec_version = int(spec['spec_version'])
    if spec_version != 1:
        raise InvalidSemanticInterface("Invalid spec version: '{0}'".format(spec_version), file_name)
    if 'name' not in spec:
        raise InvalidSemanticInterface('No name specified', file_name)
    name = spec['name']
    if 'redefines' not in spec:
        raise InvalidSemanticInterface("No redefined capability specified", file_name)
    redefines = spec['redefines']
    try:
        if isinstance(redefines, __basestring):
            validate_spec_name(redefines)
        else:
            raise InvalidSemanticInterface("Invalid redefines, must be a string", file_name)
    except (ValueError, AssertionError) as exc:
        raise InvalidSemanticInterface("Invalid spec name for redefines: " + str(exc), file_name)
    global_namespace = spec.get('global_namespace', None)
    description = spec.get('description', 'No description given.')
    try:
        semantic_capability_interface = SemanticCapabilityInterface(
            name,
            redefines,
            spec_version,
            description,
            global_namespace
        )
        semantic_capability_interface.add_remappings_by_dict(spec.get('remappings', {}))
    except (AssertionError, ValueError) as exc:
        raise InvalidSemanticInterface(str(exc), file_name)
    return semantic_capability_interface


class SemanticCapabilityInterface(object):
    """Represents a Semantic Capability Interface

    A Semantic Capability Interface is defined by:

    - name (str): name of the redefined interface
    - redefines (str): name of a capability being redefined
    - spec_type (str): type of the interface specification (has to be 'semantic_interface')
    - spec_version (int): version of the interface specification
    - description (str): free form description of the interface
    - global_namespace (str or None): (optional) global namespace for all ROS Names, None means no global_namespace
    - remappings (dict): (optional) map from ROS Names in redefined interface to Names in this interface
    """
    spec_type = 'interface'

    def __init__(self, name, redefines, spec_version, description=None, global_namespace=None):
        self.name = name
        self.redefines = redefines
        self.spec_version = spec_version
        self.default_provider = 'unknown'
        self.description = description
        self.global_namespace = global_namespace
        self.__remap_collection = RemapCollection()

    @property
    def remappings(self):
        return self.__remap_collection.remappings

    def add_remappings_by_dict(self, remappings_dict):
        self.__remap_collection.add_remappings_by_dict(remappings_dict)

    def __str__(self):
        return """Semantic Capability Interface:
{{
  name: {name}
  spec version: {spec_version}
  default provider: {default_provider}
  redefines: {redefines}
  global namespace: {global_namespace}
  description:
    {description}
}}""".format(**self.__dict__)
