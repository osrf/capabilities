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

"""This module implements some common functions used by all the spec modules
"""

from __future__ import print_function


def validate_spec_name(name):
    """Validates a given spec name follows the 'package/spec_name' format

    :param name: fully qualified spec name
    :type name: str
    :raises: AssertionError if spec name is not a str
    :raises: ValueError if spec name is invalid
    """
    split_spec_name(name)


def split_spec_name(name):
    """Splits the fully qualified spec name into package name and spec name

    :param name: fully qualified spec name
    :type name: str
    :returns: (package_name, spec_name) tuple
    :rtype: tuple
    :raises: AssertionError if spec name is not a str
    :raises: ValueError if spec name is invalid
    """
    assert isinstance(name, basestring), "a spec name must be a string"
    split_name = name.split('/')
    if len(split_name) != 2 or not split_name[0] or not split_name[1]:
        raise ValueError("Invalid spec name '{0}', it should be of the form 'package/spec_name'".format(name))
    return split_name
