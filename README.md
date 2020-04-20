segway_rmp_200 library                         {#mainpage}
============

## Description

This driver is in charge to operate motion of a SegwayRMP200 and as well as monitoring.

  The main class is SegwayRMP200,

  - The class has an FTDI driver object to ensure USB communication with the segway.
  - It also launches two threads for reading data and sending commands to the segway.
  - Uses a mutex variable, to ensure that only one thread is accessing ftdi object at a time.
	
  Segway parameters such as position and velocities paramaters are internal variables of the class, 
  and updated every 10ms approx, by the read thread.
	
  The read thread waits for a reception event coming from FTDI device. Once received it reads data stored into
  receiving buffer of FTDI device. A reference of reception event of FTDI device is retrieved at constructor so 
  as to wait upon it.

  Comanding the segway on the ohter hand is done at 50 Hz approx by the command thread, i.e one new command each
  20 ms.Actually, segway must receive a command at least every 0.4 seconds (2.5 Hz) in order to keep it moving.
  Otherwise it will automatically set motors to zero. 


## Installation

* Add the labrobotica repository if it is not already added:

Run the commands on _add repository_ and _add key_ from [labrobotica_how_to installation](https://gitlab.iri.upc.edu/labrobotica/labrobotica_how_to/-/blob/master/README.md#installation)

* Install the package:

``` sudo apt update && sudo apt install iri-segway-rmp-200-dev ```

## Scripts

A script is provided to add the Segway manufacturer devices to the dialout group. 
See [comm/scripts/add_manufacturer to_group_udev.md](https://gitlab.iri.upc.edu/labrobotica/drivers/comm/blob/install_path/scripts/add_manufacturer_to_group_udev.sh)


## Disclaimer  

Copyright (C) 2009-2018 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Mantainer IRI labrobotics (labrobotica@iri.upc.edu)

This package is distributed in the hope that it will be useful, but without any warranty. It is provided "as is" without warranty of any kind, either expressed or implied, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose. The entire risk as to the quality and performance of the program is with you. should the program prove defective, the GMR group does not assume the cost of any necessary servicing, repair  or correction.

In no event unless required by applicable law the author will be liable to you for damages, including any general, special, incidental or consequential damages arising out of the use or inability to use the program (including but not limited to loss of data or data being rendered inaccurate or losses sustained by you or third parties or a failure of the program to operate with any other programs), even if the author has been advised of the possibility of such damages.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

## For developers

<details><summary>click here</summary>
<p>

## Dependencies

This package requires of the following system libraries and packages

* [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
* [doxygen](http://www.doxygen.org "Doxygen's Homepage") and [graphviz](http://www.graphviz.org "Graphviz's Homepage") to generate the documentation.
* stdc++ and pthread libraries.

Under linux all of these utilities are available in ready-to-use packages.

This package also requires of the following IRI libraries:

* [iriutils](https://gitlab.iri.upc.edu/labrobotica/algorithms/iriutils "iriutils gitlab page"), a set of basic tools.
* [comm](https://gitlab.iri.upc.edu/labrobotica/drivers/comm "comm gitlab page"), a set of drivers for standard communication devices.

## Compilation and installation from source

Clone this repository and create a build folder inside:

``` mkdir build ```

Inside the build folder execute the following commands:

``` cmake .. ```

The default build mode is DEBUG. That is, objects and executables include debug information.

The RELEASE build mode optimizes for speed. To build in this mode execute instead
``` cmake .. -DCMAKE_BUILD_TYPE=RELEASE ```

The release mode will be kept until next time cmake is executed.

``` make -j $(nproc)``` 

In case no errors are reported, the generated libraries (if any) will be located at the
_lib_ folder and the executables (if any) will be located at the _bin_ folder.

In order to be able to use the library, it it necessary to copy it into the system.
To do that, execute

``` make install ```

as root and the shared libraries will be copied to */usr/local/lib/iri/segway_rmp_200* directory
and the header files will be copied to */usr/local/include/iri/segway_rmp_200* directory. At
this point, the library may be used by any user.

To remove the library from the system, exceute

``` make uninstall ```

as root, and all the associated files will be removed from the system.

To generate the documentation execute the following command:

``` make doc ```

## How to use it

To use this library in another library or application, in the CMakeLists.txt file, first it is necessary to locate if the library has been installed or not using the following command

``` FIND_PACKAGE(segway_rmp_200) ```

In the case that the package is present, it is necessary to add the header files directory to the include directory path by using

``` INCLUDE_DIRECTORIES(${segway_rmp_200_INCLUDE_DIR}) ```

and it is also necessary to link with the desired libraries by using the following command

``` TARGET_LINK_LIBRARIES(<executable name> ${segway_rmp_200_LIBRARY}) ```

## Examples

There are several examples that show how to use it.

</p>
</details>