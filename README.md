# Description

The notion of Capabilities in ROS is that there exists a higher level
interface for most "capabilities" that robots possess. An example of
this is the organic evolution of interfaces for things like the
Navigation Stack and the tf frame names. These sane defaults were used
on robots like the PR2 and have become defacto standards over time, and
some have even been solidified in REP's. But there has never been a
system in place to define these higher level interfaces (which consist
of other ROS concepts such as topics, services, and parameters) and to
manage their runtime in a way that makes interacting with an unknown or
abstracted robot easier.

One of the main goals of the capabilities is to make writing robot
agnostic "apps" easier. If a robot can declare that it implements the
standardized interface for the navigation capability, then an app can
rely on the ROS interface used to utilize that capability. Making it
easier for app developers to work independently of the robot developers
and making it easier to integrate onto new robots. This is analogous to
the use of Messages in topics and services, and that has proven to be
very effective at easing integration of disparate parts of the system.

# Documentation

## Tutorials

There are tutorials located here:
[Tutorials](#tutorials-1)

## API Documentation

There is fairly extensive API documentation for this package in the
CodeAPI link on the right side of this page.

The provided (Python) API can be used to discover local packages which
define one or more of the spec files defined by this package, load them,
and interact with them. It also provides an API to discover what
capabilities are provided by a `capability_server` running on a remote
machine, which is useful for tools like the
[rqt_capabilities](https://index.ros.org/p/rqt_capabilities) plugin.

## capability_server

This package provides the `capability_server` ROS node which loads all
capability spec files from packages on the `ROS_PACKAGE_PATH` and
provides a ROS interface for querying and running those capabilities.

The `capability_server` is documented in the CodeAPI (linked to on the
right).

## Design Documents

There is no organized design documentation, but there are a few
documents used during the design process that might be useful for anyone
wanting to understand the existing work:

- <https://docs.google.com/presentation/d/1sq28YcKFI8pE8WpvZUnk_jFHzbe72pgsNdwypuND7q8/edit?usp=sharing>
- <https://docs.google.com/document/d/18kAOYxSmVE1ZEQ5P1nbrJalR8vSynuqu9QFaFzxxeqY/edit?usp=sharing>
- <https://docs.google.com/document/d/1GVQ1wbrYymwp_8alyqZKRZjw0cTJ6PZAO-q1aYJBVVQ/edit?usp=sharing>

There was also an article in IEEE Spectrum (ROS Topics):

- <http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=6990859>

## Tutorials

- [Capability Demo](#capabilities-demo)

## Capabilities Demo

- **Description:** Demonstrating the capabilities of the capability
  server.
- **Keywords:** capabilities
- **Tutorial Level:** INTERMEDIATE

You can setup a demo of sorts using one of the test fixtures.



```
{#!wiki blue/solid
'''Note:''' You must install the capabilities package sources into your catkin workspace for this demo to work.
```

}

### Starting the capability server

Start the capability server with the demo.launch file:



```
$ roslaunch capabilities demo.launch
```



This is equivalent to running a roscore and then this:



```
$ rosrun capabilities capability_server `rospack find capabilities`/test/unit/discovery_workspaces/minimal
```



This starts the capability server which crawls the
``ROS_PACKAGE_PATH`` and looks for packages which export
capabilities. In this case it should find the ``minimal_pkg``
package in the ``test/discovery_workspaces`` folder. You can look at
the ``package.xml`` of ``minimal_pkg`` to see how it defines its
capabilities:



```highlight
<?xml version="1.0"?>
<package>
  ...
  <export>
    <capability_interface>
      interfaces/Minimal.yaml
    </capability_interface>
    <capability_provider>
      providers/minimal.yaml
    </capability_provider>
    <semantic_capability_interface>
      interfaces/SpecificMinimal.yaml
    </semantic_capability_interface>
    <capability_provider>
      providers/specific_minimal.yaml
    </capability_provider>

  </export>
</package>
```



It defines one capability interface, ``minimal_pkg/Minimal``, and a
semantic interface called ``minimal_pkg/SpecificMinimal``. It also
provides a capability provider for each of those interfaces. Looking at
the ``minimal_pkg/Minimal`` interface spec file:



```highlight
%YAML 1.1
---
name: Minimal
spec_version: 1
spec_type: interface
interface: {}
```



It basically fills out only the explicitly required fields (it's
minimal after all).

You can also take a look at the ``minimal_pkg/minimal`` provider:



```highlight
%YAML 1.1
---
name: minimal
spec_version: 1
spec_type: provider
implements: minimal_pkg/Minimal
launch_file: 'launch/minimal.launch'
```



You can see that this provider specifies that it implements the
``minimal_pkg/Minimal`` interface, and it has a launch file, which
is specified relatively from the ``minimal_pkg/minimal`` spec file.
This launch file simply runs a dummy script:



```highlight
<launch>
    <node name="minimal" pkg="minimal_pkg" type="minimal.py" output="screen" />
</launch>
```



Which just prints and publishes a message periodically and then stops
after 10 seconds:



```highlight
#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def minimal():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('minimal')
    start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < 10:
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        minimal()
    except rospy.ROSInterruptException:
        pass
```



### Testing out the capability server

Now there should be some key ROS services available:



```
$ rosservice list | grep /capability_server/
/capability_server/get_capability_specs
/capability_server/get_interfaces
/capability_server/get_loggers
/capability_server/get_providers
/capability_server/get_running_capabilities
/capability_server/get_semantic_interfaces
/capability_server/reload_capabilities
/capability_server/set_logger_level
/capability_server/start_capability
/capability_server/stop_capability
```



You can call these services to query, start, and stop capabilities.

You can list the available interfaces:



```
$ rosservice call /capability_server/get_interfaces
interfaces: ['minimal_pkg/Minimal']
```



You can also list the providers for an interface:



```
$ rosservice call /capability_server/get_providers minimal_pkg/Minimal
providers: ['minimal_pkg/minimal']
```



Start one with this command line:



```
$ rosservice call /capability_server/start_capability minimal_pkg/Minimal minimal_pkg/minimal
successful: True
```



This is telling the capability server to start the
``minimal_pkg/Minimal`` capability, preferring the
``minimal_pkg/minimal`` provider.

You can switch back to the ``capability_server`` and see that it is
printing the message to the screen. After 10 seconds this node should
shutdown normally, but you will get an error message like this:



```
[INFO] [WallTime: 1374023441.816336] Request to start capability 'minimal_pkg/Minimal' with provider 'minimal_pkg/minimal'
... logging to /Users/william/.ros/log/901417f8-ee7d-11e2-9b39-542696cef915/roslaunch-dosa-90541.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt

started roslaunch server http://dosa:54017/

SUMMARY
========

PARAMETERS
 * /rosdistro
 * /rosversion

NODES
  /
    minimal (minimal_pkg/minimal.py)

ROS_MASTER_URI=http://localhost:11311

core service [/rosout] found
process[minimal-1]: started with pid [90564]
[INFO] [WallTime: 1374023442.906086] hello world 1374023442.91
[INFO] [WallTime: 1374023443.907306] hello world 1374023443.91
[INFO] [WallTime: 1374023444.909204] hello world 1374023444.91
[INFO] [WallTime: 1374023445.911004] hello world 1374023445.91
[INFO] [WallTime: 1374023446.912970] hello world 1374023446.91
[INFO] [WallTime: 1374023447.914730] hello world 1374023447.91
[INFO] [WallTime: 1374023448.916598] hello world 1374023448.92
[INFO] [WallTime: 1374023449.918565] hello world 1374023449.92
[INFO] [WallTime: 1374023450.919426] hello world 1374023450.92
[INFO] [WallTime: 1374023451.921269] hello world 1374023451.92
[minimal-1] process has finished cleanly
log file: /Users/william/.ros/log/901417f8-ee7d-11e2-9b39-542696cef915/minimal-1*.log
all processes on machine have died, roslaunch will exit
shutting down processing monitor...
... shutting down processing monitor complete
done
[ERROR] [WallTime: 1374023453.342853] Capability Instance 'minimal_pkg/minimal' terminated unexpectedly, it was previously in the 'running' state.
[INFO] [WallTime: 1374023453.343502] Capability Provider 'minimal_pkg/minimal' for Capability 'minimal_pkg/Minimal' has terminated.
```



This is because from the ``capability_server``'s perspective the
launch file it ran shutdown unexpectedly. This is normal for this launch
file, but probably an error for most launch files which are designed to
run forever, until stopped.

### Stopping a capability

If you run the capability again:



```
$ rosservice call /capability_server/start_capability minimal_pkg/Minimal minimal_pkg/minimal
successful: True
```



And then within 10 seconds call:



```
$ rosservice call /capability_server/stop_capability minimal_pkg/Minimal
successful: True
```



The ``capability_server`` will preempt the provider currently
running for the ``minimal_pkg/Minimal`` interface, if one is
running. In this case we just launched the ``minimal_pkg/minimal``
provider so it will be shutdown prematurely:



```
[INFO] [WallTime: 1374018047.353614] Request to start capability 'minimal_pkg/Minimal' with provider 'minimal_pkg/minimal'
... logging to /Users/william/.ros/log/dc556d54-ee70-11e2-b171-542696cef915/roslaunch-dosa-89740.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt

started roslaunch server http://dosa:53279/

SUMMARY
========

PARAMETERS
 * /rosdistro
 * /rosversion

NODES
  /
    minimal (minimal_pkg/minimal.py)

ROS_MASTER_URI=http://localhost:11311

core service [/rosout] found
process[minimal-1]: started with pid [89763]
[INFO] [WallTime: 1374018048.422365] hello world 1374018048.42
[INFO] [WallTime: 1374018049.423824] hello world 1374018049.42
[INFO] [WallTime: 1374018050.425631] hello world 1374018050.43
[INFO] [WallTime: 1374018051.427550] hello world 1374018051.43
[INFO] [WallTime: 1374018052.428946] hello world 1374018052.43
[INFO] [WallTime: 1374018053.430748] hello world 1374018053.43
[INFO] [WallTime: 1374018054.432479] hello world 1374018054.43
[INFO] [WallTime: 1374018055.433698] hello world 1374018055.43
[INFO] [WallTime: 1374018055.752547] Request to stop capability 'minimal_pkg/Minimal'
[minimal-1] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
[INFO] [WallTime: 1374018056.651382] Capability Provider 'minimal_pkg/minimal' for Capability 'minimal_pkg/Minimal' has terminated.
```



These are the basics, more details about the specifications and chaining
capabilities in the future.
