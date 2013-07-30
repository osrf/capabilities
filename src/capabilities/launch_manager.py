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

"""This module manages the launching of capabilities"""

from __future__ import print_function

import os
import subprocess
import sys
import threading

import rospy

from capabilities.msg import CapabilityEvent


def which(program):
    """Custom versions of the ``which`` built-in shell command

    Searches the pathes in the ``PATH`` environment variable for a given
    executable name. It returns the full path to the first instance of the
    executable found or None if it was not found.

    :param program: name of the executable to find
    :type program: str
    :returns: Full path to the first instance of the executable, or None
    :rtype: str or None
    """
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ.get('PATH', os.defpath).split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file
    return None


class LaunchManager(object):
    """Manages multiple launch files which implement capabilities"""
    __roslaunch_exec = which('roslaunch')

    def __init__(self):
        self.__running_launch_files_lock = threading.Lock()
        with self.__running_launch_files_lock:
            self.__running_launch_files = {}
        self.__outputs = [sys.stdout]
        self.__event_publisher = rospy.Publisher("events", CapabilityEvent)
        self.__event_subscriber = rospy.Subscriber(
            "events", CapabilityEvent, self.handle_capability_events)
        self.stopping = False

    def stop(self):
        """Stops the launch manager, also stopping any running launch files"""
        if self.stopping:
            return
        with self.__running_launch_files_lock:
            # Incase the other thread tried the lock before this thread updated
            # the self.stopping variable, check it again.
            if self.stopping:
                return
            self.stopping = True
            for pid in self.__running_launch_files:
                self.__stop_by_pid(pid)

    def __stop_by_pid(self, pid):
        if pid not in self.__running_launch_files:
            raise RuntimeError("No running launch file with PID of '{0}'".format(pid))
        proc, thread, _, _ = self.__running_launch_files[pid]
        proc.terminate()
        proc.wait()
        thread.join()

    def stop_capability_provider(self, pid):
        """Stops the launch file for a capability provider, by pid

        :param pid: process ID of the launch file process that be stopped.
        :type pid: int
        """
        with self.__running_launch_files_lock:
            self.__stop_by_pid(pid)

    def handle_capability_events(self, msg):
        """Callback for events recieved on the events topic

        Only handles TERMINDATED events, all other events are discarded.

        :param msg: ROS message recieved on the events topic
        :type msg: :py:class:`capabilities.msgs.CapabilityEvent`
        """
        if msg.type != msg.TERMINATED:
            return
        with self.__running_launch_files_lock:
            if msg.pid in self.__running_launch_files:
                del self.__running_launch_files[msg.pid]

    def run_capability_provider(self, provider, provider_path):
        """Runs a given capability provider by launching its launch file

        :param provider: provider that should be launched
        :type provider: :py:class:`capabilities.specs.provider.CapabilityProvider`
        :param provider_path: path which the provider spec file is located at
        :type provider_path: str
        """
        if os.path.isfile(provider_path):
            provider_path = os.path.dirname(provider_path)
        launch_file = os.path.join(provider_path, provider.launch_file)
        with self.__running_launch_files_lock:
            if launch_file in [x[3] for x in self.__running_launch_files.values()]:
                raise RuntimeError("Launch file at '{0}' is already running."
                                   .format(launch_file))
            cmd = [self.__roslaunch_exec, launch_file]
            proc = subprocess.Popen(
                cmd)
            thread = self.__start_communication_thread(proc)
            msg = CapabilityEvent()
            msg.header.stamp = rospy.Time.now()
            msg.capability = provider.implements
            msg.provider = provider.name
            msg.pid = proc.pid
            msg.type = msg.LAUNCHED
            self.__running_launch_files[proc.pid] = [
                proc, thread, provider, launch_file
            ]
        self.__event_publisher.publish(msg)
        thread.start()

    def __start_communication_thread(self, proc):
        return threading.Thread(target=self.__monitor_process, args=(proc,))

    def __monitor_process(self, proc):
        try:
            with self.__running_launch_files_lock:
                if proc.pid not in self.__running_launch_files:
                    raise RuntimeError("Unknown process id: " + str(proc.pid))
                provider = self.__running_launch_files[proc.pid][2]
            if proc.stdout is not None:
                while proc.returncode is None:
                    for line in iter(proc.stdout.readline, ''):
                        for outputs in self.__outputs:
                            outputs.write(line)
            proc.wait()
            msg = CapabilityEvent()
            msg.header.stamp = rospy.Time.now()
            msg.capability = provider.implements
            msg.provider = provider.name
            msg.type = msg.TERMINATED
            msg.pid = proc.pid
            self.__event_publisher.publish(msg)
        except Exception as e:
            print(str(e), file=sys.stderr)


assert LaunchManager._LaunchManager__roslaunch_exec is not None, "'roslaunch' executable not found"
