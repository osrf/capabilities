^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package capabilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2020-03-10)
------------------
* Updated ``package.xml`` to format 3 and and used condition dependencies to support Python3 (`#91 <https://github.com/osrf/capabilities/issues/91>`_)
* Contributors: William Woodall

0.3.0 (2020-03-09)
------------------
* Bump CMake version to avoid CMP0048 warning (`#89 <https://github.com/osrf/capabilities/issues/89>`_)
* Fixed bug by pruning spec files by black/white lists `#84 <https://github.com/osrf/capabilities/issues/84>`_ from commaster90/fix-1
  This ensures ``GetCapabilitySpecs`` service returns the filtered specs.
* Removed a faulty ``start_capability`` test.
  Can't rely on timing of calling ``stop_capability`` and then ``start_capability`` in quick succession, so this test can't be made to work reliably.
* Updated discovery unit test.
* Fixed typos in ``start_capabilities`` server.
* Added test for restarting capabilities.
* Export architecture_independent flag in package.xml
* Changed to return an error if requested capability is already running.
  The ``start_capability`` service now returns an error code if it cannot start a capability because it is already running.
* Contributors: Jon Binney, Patrick Chin, Scott K Logan, Shane Loretz, William Woodall

0.2.0 (2014-06-27)
------------------
* downgrade one of the exceptions to a warning
* fixup tests to reflect changes to client API
* Increase queue_size to 1000 for publishers
* Add queue_size arg for all publishers
* change exception behavior for use/free_capability in client API
* A rosdistro agnostic documentation reference
* conditionally try to stop reverse deps, since other reverse deps may have already stopped it
* make stopping the launch manager more robust to errors
* adds support of namespaces for capability nodelets
* Contributors: Jon Binney, Marcus Liebhardt, Nikolaus Demmel, William Woodall, kentsommer

0.1.1 (2014-05-02)
------------------
* Add entry in setup.py to install package data
* Fixed up testing
* Updates link to API doc
* Contributors: Marcus Liebhardt, William Woodall

0.1.0 (2014-04-15)
------------------
* First release
* Contributors: Esteve Fernandez, Marcus Liebhardt, Tully Foote, William Woodall
