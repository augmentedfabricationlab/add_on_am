============================================================
add_on_am: Add-On AM
============================================================

.. start-badges

.. image:: https://img.shields.io/badge/License-MIT-blue.svg
    :target: https://github.com/augmentedfabricationlab/add_on_am/blob/master/LICENSE
    :alt: License MIT

.. image:: https://travis-ci.org/augmentedfabricationlab/add_on_am.svg?branch=master
    :target: https://travis-ci.org/augmentedfabricationlab/add_on_am
    :alt: Travis CI

.. end-badges

.. Write project description

**Mobile robot task structuring for extrusion-based AM onto 3D surface geometries in construction environments** ...


Main features
-------------

* Buckling analysis of a wall geometry using Karamba3D and Kiwi3D!.
* Position finding and segmentation of a wall with a recursive growth algorithm.
* Position finding and segmentation of a wall based on a path generated with the Lowest-Axis-Path method.
* Position finding and segmentation of a wall based on a path generated with the heat method.
* Simulation of the robot reaching path nodes from the found positions.

**add_on_am** runs on Rhino3D 7, with Grasshopper and IronPython 2.7 and compas 1.17.5 and compas_fab 0.28.0.


Documentation
-------------

Open the respective file in Grasshopper and run it.

For more detail see the thesis file.

Requirements
------------

Rhino3D 7

compas==1.17.5

compas_fab==0.28.0

ur_fabrication_control

Installation
------------

See installation https://augmentedfabricationlab.github.io/workshop_aaec_revamp//getting_started/


Contributing
------------

Make sure you setup your local development environment correctly:

* Clone the `add_on_am <https://github.com/augmentedfabricationlab/add_on_am>`_ repository.
* Install development dependencies and make the project accessible from Rhino:

::

    pip install -r requirements-dev.txt
    invoke add-to-rhino

**You're ready to start working!**

During development, use tasks on the
command line to ease recurring operations:

* ``invoke clean``: Clean all generated artifacts.
* ``invoke check``: Run various code and documentation style checks.
* ``invoke docs``: Generate documentation.
* ``invoke test``: Run all tests and checks in one swift command.
* ``invoke add-to-rhino``: Make the project accessible from Rhino.
* ``invoke``: Show available tasks.

For more details, check the `Contributor's Guide <CONTRIBUTING.rst>`_.


Credits
-------------

This package was created by Tim Daffner <tim.daffner@tum.de> `@Tim-2112-D <https://github.com/Tim-2112-D>`_ at `@augmentedfabricationlab <https://github.com/augmentedfabricationlab>`_
