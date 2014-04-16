The ``capability_server`` ROS Node
==================================

The ``capability_server`` ROS Node which is provided with this package uses the Python API's provided by this package to serve the discovered capability specifications and manage the run time of capabilities.

With the ``capability_server`` you can list the available capability interfaces and providers, start and stop capabilities explicitly or using reference counting calls "use" and "free".

It is recommended that you use :doc:`capabilities.client` and/or :doc:`capabilities.service_discovery` in conjunction with the ``capability_server``.

ROS API
^^^^^^^

The ``capability_server`` provides most of its functionality via ROS services and topics and is configured with ROS parameters.

ROS Parameters
--------------

Using ROS Parameters you can control what is loaded by the capability server:

- ``~package_whitelist`` (**list of strings** *Default: null*)

  If not *null* then the packages listed will be the only packages from which specs are loaded.

- ``~package_blacklist`` (**list of strings** *Default: null*)

  If not *null* then the packages listed will be ignored when loading specs.

- ``~whitelist`` (**list of strings** *Default: null*)

  If not *null* then only the specs listed will be loaded (if found), specs must have fully qualified names, i.e. ``<package_name>/<spec_name>``.

- ``~blacklist`` (**list of strings** *Default: null*)

  If not *null* then the listed specs will be ignored, specs must have fully qualified names, i.e. ``<package_name>/<spec_name>``.

The easiest way to specify the above parameters is in YAML, either in a separate file, like this:

.. code-block:: yaml

    package_whitelist:
      - package1
      - package2
    package_blacklist:
      - package2
    whitelist:
      - 'package1/SomeCapability'
      - 'package2/some_capability_provider'
      - 'package2/some_capability_provider_faux'
    blacklist:
      - 'package2/some_capability_provider_faux'

Which can be loaded in a launch file like this:

.. code-block:: xml

    <rosparam command="load" file="$(find some_package)/black_and_white_lists.yaml" />

Or inline in the launch file like this:

.. code-block:: xml

    <node pkg="capabilities" type="capability_server" name="capability_server" output="screen">
        <rosparam param="package_whitelist">
            - package1
            - package2
        </rosparam>
        <rosparam param="package_blacklist">
            - package2
        </rosparam>
        <rosparam param="whitelist">
            - 'package1/SomeCapability'
            - 'package2/some_capability_provider'
            - 'package2/some_capability_provider_faux'
        </rosparam>
        <rosparam param="blacklist">
            - 'package2/some_capability_provider_faux'
        </rosparam>
    </node>

You can also control the behavior of the ``capability_server`` using these parameters:

- ``~debug`` (**bool** *Default: false*)

  If *true* debug level messages will be printed to the screen.

- ``~missing_default_provider_is_an_error`` (**bool** *Default: false*)

  If *true* then if any interfaces with no configured default provider will result in an error instead of a warning.

- ``~nodelet_manager_name`` (**str** *Default: capability_server_nodelet_manager*)

  Name of the internal nodelet manager which is managed by the ``capability_server``.

- ``~use_screen`` (**bool** *Default: false*)

  If *true* the ``--screen`` option is passed to ``roslaunch`` when launching a capability provider's launch file.

ROS Topics
----------

There is only one ROS topic provided by the ``capability_server``:

- ``~events`` (`capabilities/CapabilityEvent <http://docs.ros.org/hydro/api/capabilities/html/msg/CapabilityEvent.html>`_)

On this topic several types of events are published and the type is set in the ``type`` field of the message:

- "server_ready": fired once when the ``capability_server`` is done loading and ready
- "launched": fired each time a new capability is launched
- "stopped": fired each time a capability is asked to shutdown
- "terminated": fired each time a capability provider's launch file terminates (expected or otherwise)

The ``capability``, ``provider``, and ``pid`` fields of the message are set only for the "launched", "stopped", and "terminated" event types.

ROS Services
------------

- ``~start_capability`` (`capabilities/StartCapability <http://docs.ros.org/hydro/api/capabilities/html/srv/StartCapability.html>`_)

Starts a given capability interface, with an optional preferred provider. If the preferred provider is not given, the default provider is used.

- ``~stop_capability`` (`capabilities/StopCapability <http://docs.ros.org/hydro/api/capabilities/html/srv/StopCapability.html>`_)

Stops a given capability interface.

- ``~use_capability`` (`capabilities/UseCapability <http://docs.ros.org/hydro/api/capabilities/html/srv/UseCapability.html>`_)

Notifies the capability server that you use this capability.
If not already running, the capability interface is started (using the preferred provider if provided), otherwise the reference count is simply incremented.

It is recommended to use :doc:`capabilities.client` instead of using this service directly.

- ``~free_capability`` (`capabilities/FreeCapability <http://docs.ros.org/hydro/api/capabilities/html/srv/FreeCapability.html>`_)

Notifies the capability server that you no longer need **one** of the "use"'s you previously made on this capability.
If the reference count goes to zero, then the capability will be stopped.

It is recommended to use :doc:`capabilities.client` instead of using this service directly.

- ``~establish_bond`` (`capabilities/EstablishBond <http://docs.ros.org/hydro/api/capabilities/html/srv/EstablishBond.html>`_)

Generates and returns the ``bond_id`` on which a bond should be established.
This ``bond_id`` is used by the ``~use_capability`` and ``~free_capability`` services.

It is recommended to use :doc:`capabilities.client` instead of using this service directly.

- ``~reload_capabilities`` (`std_srvs/Empty <http://docs.ros.org/hydro/api/std_srvs/html/srv/Empty.html>`_)

Reloads the capabilities from the ``ROS_PACKAGE_PATH``.

- ``~get_interfaces`` (`capabilities/GetInterfaces <http://docs.ros.org/hydro/api/capabilities/html/srv/GetInterfaces.html>`_)

Returns a list of strings which are the names of the available Capability Interfaces.

- ``~get_semantic_interfaces`` (`capabilities/GetSemanticInterfaces <http://docs.ros.org/hydro/api/capabilities/html/srv/GetSemanticInterfaces.html>`_)

Returns a list of strings which are the names of the available Capability Semantic Interfaces.

- ``~get_providers`` (`capabilities/GetProviders <http://docs.ros.org/hydro/api/capabilities/html/srv/GetProviders.html>`_)

Returns a list of strings which are the names of the available Capability Providers.

- ``~get_running_capabilities`` (`capabilities/GetRunningCapabilities <http://docs.ros.org/hydro/api/capabilities/html/srv/GetRunningCapabilities.html>`_)

Returns a list of `capabilities/RunningCapability <http://docs.ros.org/hydro/api/capabilities/html/msg/RunningCapability.html>`_, each of which captures the running capabilities' interface, provider, pid, and dependent capabilities.

- ``~get_capability_specs`` (`capabilities/GetCapabilitySpecs <http://docs.ros.org/hydro/api/capabilities/html/srv/GetCapabilitySpecs.html>`_)

Returns a list of `capabilities/CapabilitySpec <http://docs.ros.org/hydro/api/capabilities/html/msg/CapabilitySpec.html>`_, each of which captures all the information necessary to represent that capability specification.

It is recommended to use :doc:`capabilities.service_discovery` instead of using this service directly.

- ``~get_capability_spec`` (`capabilities/GetCapabilitySpec <http://docs.ros.org/hydro/api/capabilities/html/srv/GetCapabilitySpec.html>`_)

Returns a single `capabilities/CapabilitySpec <http://docs.ros.org/hydro/api/capabilities/html/msg/CapabilitySpec.html>`_, which captures all the information necessary to represent that capabilities specification.

- ``~get_nodelet_manager_name`` (`capabilities/GetNodeletManagerName <http://docs.ros.org/hydro/api/capabilities/html/srv/GetNodeletManagerName.html>`_)

Returns the name of the nodelet manager node for this ``capability_server``.

- ``~get_remappings`` (`capabilities/GetRemappings <http://docs.ros.org/hydro/api/capabilities/html/srv/GetRemappings.html>`_)

Returns a list of remappings for each ROS primitive (topics, services, actions, parameters) for the given capability interface.

This is useful for determining what the actual topic name is for a semantic interface or provider which has remappings.
