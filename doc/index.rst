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
