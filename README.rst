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

* Buckling analysis of a wall geometry using Karamba3D and Kiwi3D!. (requires Karamba3D / Kiwi3D!)
* Position finding and segmentation of a wall with a Recursive Growth Algorithm.
* Position finding and segmentation of a wall based on a path generated with the Lowest-Axis-Path method.
* Position finding and segmentation of a wall based on a path generated with the Heat Method.
* Simulation of the robot reaching path nodes from the found positions. (requires Docker and mobile_robot_control repository)


**add_on_am** runs on Rhino3D 7, with Grasshopper and IronPython 2.7 and compas 1.17.5 and compas_fab 0.28.0.


Documentation
-------------

Open the respective file in Grasshopper and run it.
The Rhino folder contains the main files that are discussed during the results in the thesis.
The subdirectory stability contains the Grasshopper files for non-linear analysis and eigenvalue/mode analysis of the geometry.
The simulation subdirectory contains the files for simulation of the Path. For the Simulation, the docker container from the mobile_robot_control repository must be started before opening the file. Then the robot model can be loaded from there.
segmentation_others contains the segmentation methods, that are presented in the Chapter methods of the thesis, but are not in detail discussed during the results.


For more detail see the thesis file.

Requirements
------------

Rhino3D 7

compas==1.17.5

compas_fab==0.28.0

Docker

Karamba3D
Kiwi3D!

ur_fabrication_control
mobile_robot_control

Installation
------------

See installation https://augmentedfabricationlab.github.io/workshop_aaec_revamp//getting_started/


Credits
-------------

This package was created by Tim Daffner <tim.daffner@tum.de> `@Tim-2112-D <https://github.com/Tim-2112-D>`_ at `@augmentedfabricationlab <https://github.com/augmentedfabricationlab>`_
