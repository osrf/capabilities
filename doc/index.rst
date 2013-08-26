Capabilities
============

Implements the concept of capabilities as part of the robots-in-concert system.

API Documentation
-----------------

.. toctree::

    capabilities.discovery: API for discovering and loading capability related spec files from the ROS_PACKAGE_PATH <capabilities.discovery>
    capabilities.launch_manager: provides a system for launching and managing running capability providers <capabilities.launch_manager>
    capabilities.server: Implements the capability_server command line program <capabilities.server>
    capabilities.service_discovery: Provides a method to do discovery via a ROS service call to a running capability_server <capabilities.service_discovery>
    capabilities.specs: API for loading, parsing, and interacting with various spec definitions <capabilities.specs>

Building
--------

Build it in a catkin workspace or build it stand alone:

.. code:: bash

    $ source /opt/ros/hydro/setup.bash
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ source ./devel/setup.bash

Running the Tests
-----------------

After ``source``'ing the ``setup.bash`` file generated from building you can run nosetests from the source directory:

.. code:: bash

    $ nosetests --with-coverage --cover-package capabilities -s

This is will report coverage of the non-ROS modules. You can run the rest of the tests by invoking the ``run_tests`` Make target in the build folder:

.. code:: bash

    $ cd build
    $ make && make run_tests

Running a demo
--------------

See: http://ros.org/wiki/capabilities/Tutorials