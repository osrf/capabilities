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

You can use this API as follows, (examples assume use of the
'test/discovery_workspaces/minimal' workspace::

    >>> from pprint import pprint
    >>> from capabilities.discovery import package_index_from_package_path
    >>> from capabilities.discovery import spec_file_index_from_package_index
    >>> from capabilities.discovery import spec_index_from_spec_file_index
    >>> workspaces = ['test/discovery_workspaces/minimal']
    >>> package_index = package_index_from_package_path(workspaces)
    >>> spec_file_index = spec_file_index_from_package_index(package_index)
    >>> pprint(spec_file_index)
    {'minimal_pkg': {
        'capability_interface': ['test/discovery_workspaces/minimal/minimal_pkg/interfaces/Minimal.yaml'],
        'capability_provider': [
            'test/discovery_workspaces/minimal/minimal_pkg/providers/minimal.yaml',
            'test/discovery_workspaces/minimal/minimal_pkg/providers/specific_minimal.yaml'],
        'package': <catkin_pkg.package.Package object at 0x10bb28df8>,
        'semantic_capability_interface': [
            'test/discovery_workspaces/minimal/minimal_pkg/interfaces/SpecificMinimal.yaml']}}

    >>> spec_index, errors = spec_index_from_spec_file_index(spec_file_index)
    >>> print(errors)
    []
    >>> spec_index.names
    [minimal_pkg/specific_minimal,
     minimal_pkg/Minimal,
     minimal_pkg/SpecificMinimal,
     minimal_pkg/minimal]
    >>> pprint(spec_index.specs)
    {minimal_pkg/minimal:
        <capabilities.specs.provider.CapabilityProvider object at 0x10391ce50>,
     minimal_pkg/specific_minimal:
        <capabilities.specs.provider.CapabilityProvider object at 0x10391cd10>,
     minimal_pkg/Minimal:
        <capabilities.specs.interface.CapabilityInterface object at 0x103952f90>,
     minimal_pkg/SpecificMinimal:
        <capabilities.specs.semantic_interface.SemanticCapabilityInterface object at 0x103952b50>}
    >>> spec_index.interface_names
    [minimal_pkg/Minimal]
    >>> spec_index.interfaces
    {minimal_pkg/Minimal: <capabilities.specs.interface.CapabilityInterface at 0x103952f90>}
    >>> spec_index.interfaces['Minimal']
    <capabilities.specs.interface.CapabilityInterface object at 0x10b7e3410>
    >>> spec_index.semantic_interfaces
    {'SpecificMinimal': <capabilities.specs.semantic_interface.SemanticCapabilityInterface object at 0x10b7bf3d0>}
    >>> pprint(spec_index.providers)
    {'minimal': <capabilities.specs.provider.CapabilityProvider object at 0x10b7bf750>,
     'specific_minimal': <capabilities.specs.provider.CapabilityProvider object at 0x10b7bfd10>}

"""

import os

from catkin_pkg.packages import find_packages

from capabilities.specs.interface import capability_interface_from_file_path
from capabilities.specs.interface import InvalidInterface

from capabilities.specs.provider import capability_provider_from_file_path
from capabilities.specs.provider import InvalidProvider

from capabilities.specs.semantic_interface import semantic_capability_interface_from_file_path
from capabilities.specs.semantic_interface import InvalidSemanticInterface


class DuplicateNameException(Exception):
    """Raised when multiple specs with the same name are discovered

    :ivar str spec_name: name of the spec's which collided
    :ivar str package: package in which the colliding spec is defined
    :ivar str spec_type: type of the colliding spec one of:
        capability_interface, semantic_capability_interface, capability_provider
    """
    def __init__(self, name, colliding_package, spec_type):
        self.spec_name = name
        self.package = colliding_package
        self.spec_type = spec_type
        msg = "Spec named '{0}' is defined twice in the '{1}' package."
        msg = msg.format(name, colliding_package)
        Exception.__init__(self, msg)


class InterfaceNameNotFoundException(Exception):
    """Raised when a referenced interface spec is not found

    This can happen, for example, when a provider depends on an interface
    which is not defined anywhere.

    :ivar str spec_name: name of the spec's which referenced the non existent
        interface
    :ivar str package: package in which offending spec resides
    :ivar str spec_type: type of the offending spec one of:
        capability_interface, semantic_capability_interface, capability_provider
    """
    def __init__(self, msg, spec_name, spec_type, spec_package):
        self.spec_name = spec_name
        self.package = spec_package
        self.spec_type = spec_type
        Exception.__init__(self, msg)


def package_index_from_package_path(package_paths):
    """Finds all packages on the given list of paths.

    Iterates over the given list of paths in reverse order so that packages
    found in the paths at the beginning of the list get overlaid onto packages
    with the same name which were found in paths farther back in the list.

    The resulting dictionary is keyed by the package name (so packages with
    duplicate names are overlaid) and the values are the
    :py:class:`catkin_pkg.package.Package` class

    :param list ros_package_path: list of paths to search
    :returns: dictionary of package objects keyed by name of the package
    :rtype: :py:obj:`dict`
    """
    result = {}
    for path in reversed(package_paths):
        for package_path, package in find_packages(path).items():
            result[package.name] = package
    return result


def spec_file_index_from_package_index(package_index):
    """Creates an index of spec files by package.

    Takes a dict of package objects keyed by package name.

    Returns a dict structured like this:

    .. code-block:: python

        {
            '<package_name>': {
                'package': package_obj,
                'capability_interface': ['path to spec file', ...],
                'capability_provider': ['path to spec file', ...],
                'semantic_capability_interface': ['path to spec file', ...]
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
    :rtype: :py:obj:`dict`
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
        # Prune packages with no specs
        if (
                not spec_file_index[package_name]['capability_interface']
            and not spec_file_index[package_name]['capability_provider']
            and not spec_file_index[package_name]['semantic_capability_interface']
        ):
            del spec_file_index[package_name]
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

    Goes through each spec paths foreach package of the given spec file index
    and parses them into objects. The objects are stored in a
    :py:class:`SpecIndex` before being returned.

    Duplicate Names are not allowed, even between different spec types
    and packages. Any duplicate names will be raised as a
    :py:exc:`DuplicateNameException`.

    Any other errors encountered during spec file processing will be returned
    as a list along with the :py:class:`SpecIndex`.
    Caught errors include:

    - :py:exc:`capabilities.discovery.InterfaceNameNotFoundException`
    - :py:exc:`capabilities.discovery.DuplicateNameException`
    - :py:exc:`capabilities.specs.interface.InvalidInterface`
    - :py:exc:`capabilities.specs.semantic_interface.InvalidSemanticInterface`
    - :py:exc:`capabilities.specs.provider.InvalidProvide`

    :param spec_file_index: spec_file_index, see
        :py:func:`spec_file_index_from_package_index`
    :type spec_file_index: dict
    :returns: :py:class:`SpecIndex` (which contains all the loaded specs)
        and a :py:obj:`list` of any errors encountered while loading the spec files
    :rtype: :py:class:`SpecIndex`, :py:obj:`list` (:py:obj:`Exception`'s)
    :raises: :py:exc:`DuplicateNameException` when two interfaces have the same name
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

    def add_interface(self, interface, file_path, package_name):
        """Add a loaded CapabilityInterface object into the repository

        Used by :py:func:`spec_index_from_spec_file_index` to build the :py:class:`SpecIndex`.

        :param interface: CapabilityInterface object which was loaded using a
            factory function
        :type interface: :py:class:`.specs.interface.CapabilityInterface`
        :param str file_path: path to the interface spec file that was loaded
        :param str package_name: name of the package which contains the interface
        :raises: :py:exc:`DuplicateNameException` if there is a name collision
        """
        interface_name = '{package}/{name}'.format(package=package_name, name=interface.name)
        interface.name = interface_name
        if interface_name in self.names:
            raise DuplicateNameException(
                interface_name, package_name,
                'capability_interface')
        self.__add_package(package_name)
        self.__interfaces[interface_name] = {
            'path': file_path,
            'instance': interface
        }

    def remove_interface(self, interface_name):
        """Removes a capability interface by name

        :param str interface_name: name of the interface to remove
        :raises: :py:exc:`KeyError` if there is no interface by that name
        """
        del self.__interfaces[interface_name]

    def add_semantic_interface(self, semantic_interface, file_path, package_name):
        """Add a loaded SemanticCapabilityInterface object into the repository

        Used by :py:func:`spec_index_from_spec_file_index` to build the :py:class:`SpecIndex`.

        :param semantic_interface: SemanticCapabilityInterface object which was
            loaded using a factory function
        :type semantic_interface:
            :py:class:`.specs.semantic_interface.SemanticCapabilityInterface`
        :param str file_path: path to the semantic interface spec file that
            was loaded
        :param str package_name: name of the package which contains the
            semantic interface
        :raises: :py:exc:`DuplicateNameException` if there is a name collision
        :raises: :py:exc:`InterfaceNameNotFoundException` if the interface which
            this semantic capability interface redefines is not found.
        """
        semantic_interface_name = '{package}/{name}'.format(package=package_name, name=semantic_interface.name)
        semantic_interface.name = semantic_interface_name
        if semantic_interface_name in self.names:
            raise DuplicateNameException(
                semantic_interface_name, package_name,
                'semantic_capability_interface')
        if semantic_interface.redefines not in self.interface_names:
            raise InterfaceNameNotFoundException(
                "Semantic capability interface '{0}' redefines '{1}', but the '{1}' interface was not found."
                .format(semantic_interface_name, semantic_interface.redefines),
                semantic_interface_name, package_name,
                'semantic_capability_interface')
        self.__add_package(package_name)
        self.__semantic_interfaces[semantic_interface_name] = {
            'path': file_path,
            'instance': semantic_interface
        }

    def remove_semantic_interface(self, semantic_interface_name):
        """Removes a semantic capability interface by name

        :param str semantic_interface_name: name of the interface to remove
        :raises: :py:exc:`KeyError` if there is no interface by that name
        """
        del self.__semantic_interfaces[semantic_interface_name]

    def add_provider(self, provider, file_path, package_name):
        """Add a loaded CapabilityProvider object into the repository

        Used by :py:func:`spec_index_from_spec_file_index` to build the :py:class:`SpecIndex`.

        :param provider: CapabilityProvider object which was loaded using a
            factory function
        :type provider: :py:class:`.specs.provider.CapabilityProvider`
        :param str file_path: path to the provider spec file that was loaded
        :param str package_name: name of the package which contains the provider
        :raises: :py:exc:`DuplicateNameException` if there is a name collision
        :raises: :py:exc:`InterfaceNameNotFoundException` if the interface which
            this capability provider implements is not found.
        """
        provider_name = '{package}/{name}'.format(package=package_name, name=provider.name)
        provider.name = provider_name
        if provider_name in self.names:
            raise DuplicateNameException(
                provider_name, package_name,
                'capability_provider')
        interfaces = (self.interface_names + self.semantic_interface_names)
        if provider.implements not in interfaces:
            raise InterfaceNameNotFoundException(
                "Capability provider '{0}' implements '{1}', but the '{1}' interface was not found."
                .format(provider_name, provider.implements),
                provider_name, package_name,
                'capability_provider')
        self.__add_package(package_name)
        self.__providers[provider_name] = {
            'path': file_path,
            'instance': provider
        }

    def remove_provider(self, provider_name):
        """Removes a capability provider by name

        :param str provider_name: name of the interface to remove
        :raises: :py:exc:`KeyError` if there is no interface by that name
        """
        del self.__providers[provider_name]

    @property
    def names(self):
        """
        :returns: list of the names for all specs of all types
        :rtype: :py:obj:`list` (:py:obj:`str`)
        """
        return self.interfaces.keys() + self.semantic_interfaces.keys() + self.providers.keys()

    @property
    def specs(self):
        """
        Possible spec types:

        - :py:class:`.specs.interface.CapabilityInterface`
        - :py:class:`.specs.semantic_interface.SemanticCapabilityInterface`
        - :py:class:`.specs.provider.CapabilityProvider`

        :returns: dict of specs, keyed by name
        :rtype: :py:obj:`dict` {:py:obj:`str`: spec}
        """
        result = {}
        # There should be no key collisions as collisions are found on insertion
        result.update(self.interfaces)
        result.update(self.semantic_interfaces)
        result.update(self.providers)
        return result

    @property
    def interface_names(self):
        """
        :returns: list of capability interface names
        :rtype: :py:obj:`list` (:py:obj:`str`)
        """
        return [n for n in self.__interfaces.keys()]

    @property
    def interfaces(self):
        """
        :returns: dict of capability interfaces, keyed by name
        :rtype: :py:obj:`dict` {:py:obj:`str`: :py:class:`.specs.interface.CapabilityInterface`}
        """
        return dict([(n, x['instance']) for n, x in self.__interfaces.items()])

    @property
    def interface_paths(self):
        """
        :returns: dict of capability interface spec paths, keyed by name
        :rtype: :py:obj:`dict` {:py:obj:`str`: :py:obj:`str`}
        """
        return dict([(n, x['path']) for n, x in self.__interfaces.items()])

    @property
    def provider_names(self):
        """
        :returns: list of capability provider names
        :rtype: :py:obj:`list` (:py:obj:`str`)
        """
        return [n for n in self.__providers.keys()]

    @property
    def providers(self):
        """
        :returns: dict of capability providers, keyed by name
        :rtype: :py:obj:`dict` {:py:obj:`str`: :py:class:`.specs.provider.CapabilityProvider`}
        """
        return dict([(n, x['instance']) for n, x in self.__providers.items()])

    @property
    def provider_paths(self):
        """
        :returns: dict of capability provider spec paths, keyed by name
        :rtype: :py:obj:`dict` {:py:obj:`str`: :py:obj:`str`}
        """
        return dict([(n, x['path']) for n, x in self.__providers.items()])

    @property
    def semantic_interface_names(self):
        """
        :returns: list of semantic capability interface names
        :rtype: :py:obj:`list` (:py:obj:`str`)
        """
        return [n for n in self.__semantic_interfaces.keys()]

    @property
    def semantic_interfaces(self):
        """
        :returns: dict of semantic capability interfaces, keyed by name
        :rtype: :py:obj:`dict` {:py:obj:`str`: :py:class:`.specs.semantic_interface.SemanticCapabilityInterface`}
        """
        return dict([(n, x['instance']) for n, x in self.__semantic_interfaces.items()])

    @property
    def semantic_interface_paths(self):
        """
        :returns: dict of semantic capability interface spec paths, keyed by name
        :rtype: :py:obj:`dict` {:py:obj:`str`: :py:obj:`str`}
        """
        return dict([(n, x['path']) for n, x in self.__semantic_interfaces.items()])
