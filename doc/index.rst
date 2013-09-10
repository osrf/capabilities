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

After ``source``'ing the ``setup.bash`` file generated from building you can run nosetests from the source directory:

.. code-block:: bash

    $ nosetests --with-coverage --cover-package capabilities -s

This is will report coverage of the non-ROS modules. You can run the rest of the tests by invoking the ``run_tests`` Make target in the build folder:

.. code-block:: bash

    $ cd build
    $ make && make run_tests

Running a demo
--------------

See: http://ros.org/wiki/capabilities/Tutorials