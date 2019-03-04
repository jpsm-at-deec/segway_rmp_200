segway rmp 200 library                         {#mainpage}
============

## Description

This driver offers a c++ wrapper to control a Segway RMP 200 platform.

## Dependencies

This package requires of the following system libraries and packages

 * [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
 * [doxygen](http://www.doxygen.org "Doxygen's Homepage") and [graphviz](http://www.graphviz.org "Graphviz's Homepage") to generate the documentation.
 * stdc++.

Under linux all of these utilities are available in ready-to-use packages.

Under MacOS most of the packages are available via [fink](http://www.finkproject.org/ "Fink's Homepage")

This package also requires of the following IRI libraries:

 * [iriutils](https://gitlab.iri.upc.edu/labrobotica/algorithms/iriutils "iriutils gitlab page"), a set of basic tools.
 * [comm](https://gitlab.iri.upc.edu/labrobotica/drivers/comm "comm gitlab page"), a set of drivers for standard communication devices.

## Compilation and installation

Download this repository and create a build folder inside:

``` mkdir build ```

Inside the build folder execute the following commands:

``` cmake .. ```

The default build mode is DEBUG. That is, objects and executables include debug information.

The RELEASE build mode optimizes for speed. To build in this mode execute instead
``` cmake .. -DCMAKE_BUILD_TYPE=RELEASE ```

The release mode will be kept until next time cmake is executed.

``` make ``` 

In case no errors are reported, the generated libraries (if any) will be located at the
_lib_ folder and the executables (if any) will be located at the _bin_ folder.

In order to be able to use the library, it it necessary to copy it into the system.
To do that, execute

``` make install ```

as root and the shared libraries will be copied to */usr/local/lib/iridrivers* directory
and the header files will be copied to */usr/local/include/iridrivers* dierctory. At
this point, the library may be used by any user.

To remove the library from the system, exceute

``` make uninstall ```

as root, and all the associated files will be removed from the system.

To generate the documentation execute the following command:

``` make doc ```

## How to use it

To use this library in an other library or application, in the CMakeLists.txt file, first it is necessary to locate if the library has been installed or not using the following command

``` FIND_PACKAGE(<library name>) ```

In the case that the package is present, it is necessary to add the header files directory to the include directory path by using

``` INCLUDE_DIRECTORIES(${<librray name>_INCLUDE_DIR}) ```

and it is also necessary to link with the desired libraries by using the following command

``` TARGET_LINK_LIBRARIES(<executable name> ${<library name>_LIBRARY}) ```

## Disclaimer  

Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author shernand (shernand@iri.upc.edu)
All rights reserved.

This file is part of segway_rmp_200 driver library
segway_rmp_200 driver library is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>



