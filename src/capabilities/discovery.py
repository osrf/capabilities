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
This module implements discovery of packages which export various spec files.

You can use this API as follows, assuming workspace of
'test/discovery_workspaces/minimal'::

    >>> from pprint import pprint
    >>> from capabilities.discovery import package_index_from_package_path
    >>> from capabilities.discovery import spec_file_index_from_package_index
    >>> from capabilities.discovery import spec_index_from_spec_file_index
    >>> workspaces = ['test/discovery_workspaces/minimal']
    >>> package_index = package_index_from_package_path(workspaces)
    >>> spec_file_index = spec_file_index_from_packages_dict(package_index)
    >>> pprint(spec_file_index)
    {'minimal': {'capability_interface': ['test/discovery_workspaces/minimal/minimal/interfaces/Minimal.yaml'],
                 'capability_provider': ['test/discovery_workspaces/minimal/minimal/providers/minimal.yaml',
                                         'test/discovery_workspaces/minimal/minimal/providers/specific_minimal.yaml'],
                 'package': <catkin_pkg.package.Package object at 0x10c13e3e8>,
                 'semantic_capability_interface': ['test/discovery_workspaces/minimal/minimal/interfaces/SpecificMinimal.yaml']}}

    >>> spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
    >>> print(errors)
    []
    >>> spec_index.names
    ['Minimal', 'specific_minimal', 'minimal', 'SpecificMinimal']
    >>> pprint(spec_index.specs)
    {'Minimal': <capabilities.specs.interface.CapabilityInterface object at 0x10b7e3410>,
     'SpecificMinimal': <capabilities.specs.semantic_interface.SemanticCapabilityInterface object at 0x10b7bf3d0>,
     'minimal': <capabilities.specs.provider.CapabilityProvider object at 0x10b7bf750>,
     'specific_minimal': <capabilities.specs.provider.CapabilityProvider object at 0x10b7bfd10>}
    >>> spec_index.interface_names
    ['Minimal']
    >>> spec_index.interfaces
    {'Minimal': <capabilities.specs.interface.CapabilityInterface object at 0x10b7e3410>}
    >>> spec_index.interfaces['Minimal']
    <capabilities.specs.interface.CapabilityInterface object at 0x10b7e3410>
    >>> spec_index.semantic_interfaces
    {'SpecificMinimal': <capabilities.specs.semantic_interface.SemanticCapabilityInterface object at 0x10b7bf3d0>}
    >>> pprint(spec_index.providers)
    {'minimal': <capabilities.specs.provider.CapabilityProvider object at 0x10b7bf750>,
     'specific_minimal': <capabilities.specs.provider.CapabilityProvider object at 0x10b7bfd10>}

"""

import os

import rospy

from catkin_pkg.packages import find_packages

from capabilities.specs.interface import capability_interface_from_file_path
from capabilities.specs.interface import capability_interface_from_string
from capabilities.specs.interface import InvalidInterface

from capabilities.specs.provider import capability_provider_from_file_path
from capabilities.specs.provider import capability_provider_from_string
from capabilities.specs.provider import InvalidProvider

from capabilities.specs.semantic_interface import semantic_capability_interface_from_file_path
from capabilities.specs.semantic_interface import semantic_capability_interface_from_string
from capabilities.specs.semantic_interface import InvalidSemanticInterface

from capabilities.srv import GetCapabilitySpecs


class DuplicateNameException(Exception):
    def __init__(self, name, colliding_package, existing_package, spec_type):
        self.spec_name = name
        self.package = colliding_package
        self.spec_type = spec_type
        if colliding_package == existing_package:
            msg = "Spec named '{0}' is defined twice in the '{1}' package."
            msg = msg.format(name, colliding_package)
        else:
            msg = "Spec named '{0}' in the '{1}' package is also defined in the '{2}' package."
            msg = msg.format(name, colliding_package, existing_package)
        Exception.__init__(self, msg)


class InterfaceNameNotFoundException(Exception):
    def __init__(self, msg, spec_name, spec_type, spec_package):
        self.spec_name = spec_name
        self.package = spec_package
        self.spec_type = spec_type
        Exception.__init__(self, msg)


def package_index_from_package_path(package_paths):
    """Find all packages on the given list of paths

    Iterates over the given list of paths in reverse order so that packages
    found in the paths at the beginning of the list get overlaid onto packages
    with the same name which were found in paths farther back in the list.

    The resulting dictionary is keyed by the package name (so packages with
    duplicate names are overlaid) and the values are the
    :py:class:`catkin_pkg.package.Package` class

    :param ros_package_path: list of paths to search
    :type ros_package_path: list
    :returns: dictionary of package objects keyed by name of the package
    :rtype: dict
    """
    result = {}
    for path in reversed(package_paths):
        for package_path, package in find_packages(path).items():
            result[package.name] = package
    return result


def spec_file_index_from_package_index(package_index):
    """Creates an index of spec files by package.

    Takes a dict of package objects keyed by package name.

    Returns a dict structured like this::

        {
            '<package_name>': {
                'package': package_obj,
                'capability_interface': [path to spec file, ...],
                'capability_provider': [path to spec file, ...],
                'semantic_capability_interface': [path to spec file, ...]
            },
            ...
        }

    This dict contains a dict for each package, keyed by package name.
    Those dicts contain the parsed package object, and a list of relative paths
    for spec files, separated by spec type.

    :param package_index: dict of :py:class:`catkin_pkg.package.Package`'s
        keyed by package name to be processed
    :type package_index: dict
    :returns: spec file index strucutre
    :rtype: dict
    """
    spec_file_index = {}
    for package_name, package in package_index.items():
        spec_file_index[package_name] = {
            'package': package,
            'capability_interface': [],
            'capability_provider': [],
            'semantic_capability_interface': []
        }
        package_path = os.path.dirname(package.filename)
        for export in package.exports:
            tag = export.tagname
            if tag != 'package' and tag in spec_file_index[package_name]:
                spec_file_path = os.path.join(package_path, export.content)
                spec_file_index[package_name][tag].append(spec_file_path)
    return spec_file_index


def _spec_loader(spec_thing_index, spec_thing_loaders):
    spec_index = SpecIndex()
    errors = []
    error_types = (
        InterfaceNameNotFoundException,
        DuplicateNameException,
        InvalidInterface,
        InvalidSemanticInterface,
        InvalidProvider
    )
    # First load and process CapabilityInterface's
    for package_name, package_dict in spec_thing_index.items():
        interface_things = package_dict['capability_interface']
        for thing in interface_things:
            try:
                spec_thing_loaders['capability_interface'](thing, package_name, spec_index)
            except error_types as e:
                errors.append(e)
    # Then load the SemanticCapabilityInterface's
    for package_name, package_dict in spec_thing_index.items():
        semantic_interface_things = package_dict['semantic_capability_interface']
        for thing in semantic_interface_things:
            try:
                spec_thing_loaders['semantic_capability_interface'](thing, package_name, spec_index)
            except error_types as e:
                errors.append(e)
    # Finally load the CapabilityProvider's
    for package_name, package_dict in spec_thing_index.items():
        capability_provider_things = package_dict['capability_provider']
        for thing in capability_provider_things:
            try:
                spec_thing_loaders['capability_provider'](thing, package_name, spec_index)
            except error_types as e:
                errors.append(e)
    return spec_index, errors


def spec_index_from_spec_file_index(spec_file_index):
    """Builds a :py:class:`SpecIndex` from a spec file index

    Goes through each spec path in each package of the given spec file index
    and parses them into objects. The objects are stored in a
    :py:class:`SpecIndex` before being returned.

    Duplicate Names are not allowed, even between different spec types
    and packages. Any duplicate names will be raised as a
    :py:exc:`DuplicateNameException`.

    Any other errors encountered during spec file processing will be returned
    as a list along with the :py:class:`SpecIndex`.

    :param spec_file_index: spec_file_index, see
        :py:func:`spec_file_index_from_packages_dict`
    :type spec_file_index: dict
    :returns: SpecIndex which contains all the loaded specs
        and a list of any errors encountered while loading the spec files
    :rtype: :py:class:`SpecIndex`, :py:obj:`list`
    :raises DuplicateNameException: when two interfaces have the same name
    """
    def capability_interface_loader(path, package_name, spec_index):
        interface = capability_interface_from_file_path(path)
        spec_index.add_interface(interface, path, package_name)

    def semantic_capability_loader(path, package_name, spec_index):
        si = semantic_capability_interface_from_file_path(path)
        spec_index.add_semantic_interface(si, path, package_name)

    def capability_provider_loader(path, package_name, spec_index):
        provider = capability_provider_from_file_path(path)
        spec_index.add_provider(provider, path, package_name)

    return _spec_loader(spec_file_index, {
        'capability_interface': capability_interface_loader,
        'semantic_capability_interface': semantic_capability_loader,
        'capability_provider': capability_provider_loader
    })


def spec_index_from_service():
    """Builds a :py:class:`SpecIndex` by calling a ROS service to get the specs

    Works just like :py:func:`spec_index_from_spec_file_index`, except the raw
    spec files are retreived over a service call rather than from disk.

    :raises: :py:class:`rospy.ServiceException` when the service call fails
    """
    rospy.wait_for_service('get_capability_specs')
    get_capability_specs = rospy.ServiceProxy('get_capability_specs', GetCapabilitySpecs)
    response = get_capability_specs()
    spec_raw_index = {}
    for spec in response.capability_specs:
        package_dict = spec_raw_index.get(spec.package, {
            'capability_interface': [],
            'semantic_capability_interface': [],
            'capability_provider': []
        })
        package_dict[spec.type].append(spec.content)
        spec_raw_index[spec.package] = package_dict

    def capability_interface_loader(raw, package_name, spec_index):
        interface = capability_interface_from_string(raw)
        spec_index.add_interface(interface, 'service call', package_name)

    def semantic_capability_loader(raw, package_name, spec_index):
        si = semantic_capability_interface_from_string(raw)
        spec_index.add_semantic_interface(si, 'service call', package_name)

    def capability_provider_loader(raw, package_name, spec_index):
        provider = capability_provider_from_string(raw)
        spec_index.add_provider(provider, 'service call', package_name)

    return _spec_loader(spec_raw_index, {
        'capability_interface': capability_interface_loader,
        'semantic_capability_interface': semantic_capability_loader,
        'capability_provider': capability_provider_loader
    })


class SpecIndex(object):
    """Container for capability spec file locations and respective spec classes
    """
    def __init__(self):
        self.__packages = []
        self.__interfaces = {}
        self.__providers = {}
        self.__semantic_interfaces = {}

    def __add_package(self, package_name):
        if package_name in self.__packages:
            return
        self.__packages.append(package_name)
        self.__interfaces[package_name] = {}
        self.__providers[package_name] = {}
        self.__semantic_interfaces[package_name] = {}

    def add_interface(self, interface, file_path, package_name):
        """Add a loaded CapabilityInterface object into the repository

        :param interface: CapabilityInterface object which was loaded using a
            factory function
        :type interface: :py:class:`.specs.interface.CapabilityInterface`
        :param file_path: path to the interface spec file that was loaded
        :type file_path: str
        :param package_name: name of the package which contains the interface
        :type package_name: str
        :raises: :py:exc:`DuplicateNameException` if there is a name collision
        """
        if interface.name in self.names:
            raise DuplicateNameException(
                interface.name, package_name,
                self.get_containing_package_name(interface.name),
                'capability_interface')
        self.__add_package(package_name)
        self.__interfaces[package_name][interface.name] = {
            'path': file_path,
            'instance': interface
        }

    def add_semantic_interface(self, semantic_interface, file_path, package_name):
        """Add a loaded SemanticCapabilityInterface object into the repository

        :param semantic_interface: SemanticCapabilityInterface object which was
            loaded using a factory function
        :type semantic_interface:
            :py:class:`.specs.semantic_interface.SemanticCapabilityInterface`
        :param file_path: path to the semantic interface spec file that
            was loaded
        :type file_path: str
        :param package_name: name of the package which contains the
            semantic interface
        :type package_name: str
        :raises: :py:exc:`DuplicateNameException` if there is a name collision
        :raises: :py:exc:`InterfaceNameNotFoundException` if the interface which
            this semantic capability interface redefines is not found.
        """
        if semantic_interface.name in self.names:
            raise DuplicateNameException(
                semantic_interface.name, package_name,
                self.get_containing_package_name(semantic_interface.name),
                'semantic_capability_interface')
        if semantic_interface.redefines not in self.interface_names:
            raise InterfaceNameNotFoundException(
                "Semantic capability interface '{0}' redefines '{1}', but the '{1}' interface was not found."
                .format(semantic_interface.name, semantic_interface.redefines),
                semantic_interface.name, package_name,
                'semantic_capability_interface')
        self.__add_package(package_name)
        self.__semantic_interfaces[package_name][semantic_interface.name] = {
            'path': file_path,
            'instance': semantic_interface
        }

    def add_provider(self, provider, file_path, package_name):
        """Add a loaded CapabilityProvider object into the repository

        :param provider: CapabilityProvider object which was loaded using a
            factory function
        :type provider: :py:class:`.specs.provider.CapabilityProvider`
        :param file_path: path to the provider spec file that was loaded
        :type file_path: str
        :param package_name: name of the package which contains the provider
        :type package_name: str
        :raises: :py:exc:`DuplicateNameException` if there is a name collision
        :raises: :py:exc:`InterfaceNameNotFoundException` if the interface which
            this capability provider implements is not found.
        """
        if provider.name in self.names:
            raise DuplicateNameException(
                provider.name, package_name,
                self.get_containing_package_name(provider.name),
                'capability_provider')
        interfaces = (self.interface_names + self.semantic_interface_names)
        if provider.implements not in interfaces:
            raise InterfaceNameNotFoundException(
                "Capability provider '{0}' implements '{1}', but the '{1}' interface was not found."
                .format(provider.name, provider.implements),
                provider.name, package_name,
                'capability_provider')
        self.__add_package(package_name)
        self.__providers[package_name][provider.name] = {
            'path': file_path,
            'instance': provider
        }

    def get_containing_package_name(self, spec_name):
        """Returns the name of the package which contains the given spec

        :param spec_name: name of the spec to search for
        :type spec_name: str
        :returns: name of the package containing the spec
        :rtype: str
        :raises: :py:exc:`RuntimeError` if the spec name is not found
        """
        if spec_name not in self.names:
            raise RuntimeError("Spec by the name '{0}' not found.".format(spec_name))
        for spec_dict in [self.__interfaces, self.__semantic_interfaces, self.__providers]:
            for package_name, package_dict in spec_dict.items():
                if spec_name in package_dict:
                    return package_name

    @property
    def names(self):
        """list of all names"""
        return self.specs.keys()

    @property
    def specs(self):
        """dict of specs, keyed by name"""
        result = {}
        # There should be no key collisions as collisions are found on insertion
        result.update(self.interfaces)
        result.update(self.semantic_interfaces)
        result.update(self.providers)
        return result

    @property
    def interface_names(self):
        """list of capability interface names"""
        return [n for i in self.__interfaces.values() for n in i.keys()]

    @property
    def interfaces(self):
        """dict of capability interfaces, keyed by name"""
        return dict([(n, x['instance'])
                     for i in self.__interfaces.values() for n, x in i.items()])

    @property
    def interface_paths(self):
        """dict of capability interface spec paths, keyed by name"""
        return dict([(n, x['path'])
                     for i in self.__interfaces.values() for n, x in i.items()])

    @property
    def provider_names(self):
        """list of capability provider names"""
        return [n for p in self.__providers.values() for n in p.keys()]

    @property
    def providers(self):
        """dict of capability providers, keyed by name"""
        return dict([(n, x['instance'])
                    for p in self.__providers.values() for n, x in p.items()])

    @property
    def provider_paths(self):
        """dict of capability provider spec paths, keyed by name"""
        return dict([(n, x['path'])
                     for i in self.__providers.values() for n, x in i.items()])

    @property
    def semantic_interface_names(self):
        """list of semantic capability interface names"""
        return [n
                for si in self.__semantic_interfaces.values()
                for n in si.keys()]

    @property
    def semantic_interfaces(self):
        """dict of semantic capability interfaces, keyed by name"""
        return dict([(n, x['instance'])
                    for si in self.__semantic_interfaces.values()
                    for n, x in si.items()])

    @property
    def semantic_interface_paths(self):
        """dict of semantic capability interface spec paths, keyed by name"""
        return dict([(n, x['path'])
                     for i in self.__semantic_interfaces.values()
                     for n, x in i.items()])
