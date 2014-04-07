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
This module implements a collection of remapped ROS Names
"""

from __future__ import print_function


class RemapCollection(object):
    """Models the remappings between interfaces"""
    valid_remapping_types = ['topics', 'services', 'parameters', 'actions']

    def __init__(self):
        self.__remapped_topics = {}
        self.__remapped_services = {}
        self.__remapped_actions = {}
        self.__remapped_parameters = {}
        self.__type_mapping = {
            'topics': self.__remapped_topics,
            'services': self.__remapped_services,
            'actions': self.__remapped_actions,
            'parameters': self.__remapped_parameters
        }

    def __str__(self):
        remapping_strs = ["'{0}' -> '{1}'".format(f, t) for f, t in self.remappings.items()]
        return "remappings:\n{0}".format("\n".join(['  ' + s for s in remapping_strs]))

    @property
    def remappings(self):
        return dict([(k, v) for remaps in self.__type_mapping.values() for k, v in remaps.items()])

    @property
    def remappings_by_type(self):
        return dict(self.__type_mapping)

    def add_remapping(self, mapping_type, map_from, map_to):
        # Assert that the mapping_type is valid
        if mapping_type not in self.valid_remapping_types:
            raise ValueError("Invalid remapping type '{0}', should be one of: '{1}'"
                             .format(mapping_type, "', '".join(self.valid_remapping_types)))
        # Assert that if the map_from is already in the flat remapping list then the map_to's match
        if map_from in self.remappings:
            assert (self.remappings[map_from] == map_to), (
                "'{0}' is remapped twice, but to different values '{1}' and '{2}'"
                .format(map_from, self.remappings[map_from], map_to))
        else:
            self.__type_mapping[mapping_type][map_from] = map_to

    def add_remappings_by_dict(self, remappings_dict):
        for mapping_type, remappings in remappings_dict.items():
            for src, dst in remappings.items():
                self.add_remapping(mapping_type, src, dst)
