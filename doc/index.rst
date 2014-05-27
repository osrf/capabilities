Capabilities
============

.. toctree::
    :hidden:

    capability_server
    capabilities.client
    capabilities.discovery
    capabilities.service_discovery
    capabilities.specs
    capabilities.server
    capabilities.launch_manager

Implements the concept of capabilities as part of the robots-in-concert system.

API Documentation
-----------------

For end users it is recommended to use the ``capability_server`` script in combination with the :py:class:`capabilities.client.CapabilitiesClient` Class of the :doc:`capabilities.client <capabilities.client>` module and the :doc:`capabilities.service_discovery <capabilities.service_discovery>` module.

The ``capability_server``, along with its ROS API, is documented here: :doc:`capability_server`

The :py:class:`capabilities.client.CapabilitiesClient` class provides a simple Python class for interacting with a remote ``capability_server``, allowing you to "use" and "free" capabilities using the reference counting services provided by the server.
The client API ensures the "uses" of capabilities do not leak by maintaining a bond with each client, and it manages underlying bond creation and maintenance under the hood so that the client never sees that.
Since there is a bond with each client, if that bond breaks for any reason, then the corresponding "used" capabilities will be freed and may shutdown.
Therefore, if you want to make sure the capabilities keep running, you need to keep the :py:class:`capabilities.client.CapabilitiesClient` Class around for the lifetime of your application.

The :doc:`capabilities.service_discovery <capabilities.service_discovery>` module provides a mechanism to remotely get an instance of the powerful :py:class:`capabilities.discovery.SpecIndex` Class which allows you to introspect the available capability specs (interfaces, semantic interfaces, and providers) to which the server has access.

These are considered to be the public API for the ``capabilities`` Python package:

- :doc:`capabilities.client`: interact remotely with a capability_server
- :doc:`capabilities.discovery`: discover and load capability spec files from the ROS_PACKAGE_PATH
- :doc:`capabilities.service_discovery`: get a SpecIndex from a remote capability_server
- :doc:`capabilities.specs`: package for interacting with different types of capability specs

The rest of the documented API is used internally and not considered public, so use it at your on risk:

- :doc:`capabilities.server`: implementation of the capability_server ROS Node
- :doc:`capabilities.launch_manager`: implements a manager for the running roslaunch instances

Building
--------

Build it in a catkin workspace or build it stand alone (replace hydro with your ros version if necessary):

.. code-block:: bash

    $ source /opt/ros/hydro/setup.bash
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ source ./devel/setup.bash

Building the Documentation
--------------------------

First you need ``sphinx`` installed, on Ubuntu:

.. code-block:: bash

    $ sudo apt-get install python-sphinx

On other platforms use pip:

.. code-block:: bash

    $ sudo pip install Sphinx

You have to have built the package first, then you mush source the resulting devel or install space:

.. code-block:: bash

    $ source /path/to/space/setup.bash

Then from the capabilities source folder you can build the docs:

.. code-block:: bash

    $ cd docs
    $ make html

The resulting docs will be generated to ``doc/.build/html/index.html``.

Running the Tests
-----------------

To run the tests you will need the ``nosetests`` and ``coverage`` python packages. On Ubuntu you can get these like this:

.. code-block:: bash

    $ sudo apt-get install python-nose python-coverage

On other platforms you can use ``pip``:

.. code-block:: bash

    $ sudo pip install nose coverage

You will also need the build and run dependencies for capabilities because it must be built during the testing.
You can get the dependencies you need using ``rosdep``:

.. code-block:: bash

    $ rosdep install --from-paths ./ --ignore-src --rosdistro hydro -y

Remember to update the value given to ``--rosdistro`` to the ROS distro you are using and to change the ``./`` given to ``--from-paths`` if you are not in the local checkout of the ``capabilities`` source code.

Finally you can run the tests with a coverage report by invoking the ``coverage`` target of the provided ``Makefile`` in the root of the capabilities source repository:

.. code-block:: bash

    $ make coverage

Running a demo
--------------

See: http://wiki.ros.org/capabilities/Tutorials
