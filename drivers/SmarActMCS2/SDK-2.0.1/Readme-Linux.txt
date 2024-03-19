            ================================
                 MCS2 SDKs for Linux
                (c) 2020 by SmarAct GmbH
            ================================


REQUIREMENTS
============

This version of the MCS2 SDKs is compatible with Linux for 32 
bit or 64 bit x86 architecture.

For installing the Python SDK, additional requirements must be
met which are described in doc/Using SmarAct Python SDKs.pdf.


LICENSE
=======

When installing the software, the SmarAct Software Licensing
Agreement (EULA) must be accepted. By passing "--license accept"
to the installer script, you confirm that you have read and agreed
to the EULA. The EULA text should be included in this software
package. If it is not included, please contact SmarAct.


INSTALLATION
============

The archive MCS2_<version>_Linux.tgz can be unpacked in a
directory of your choice.

In the unpacked folder, the install_mcs2.sh script is used
to install the software components on your computer. 

    install_mcs2.sh -h 

shows the help text of the installer.

The MCS2 software requires several libraries to be installed:
libsmaractctl and libsmaractio which are installed if either
--sdk-c or --libraries is selected.
If your MCS2 controller is connected via USB to your computer, the
additional 3rd party library libftd2xx must be installed.
This is the default installer behavior. 
If you want to suppress the installation of the USB support libraries, 
you can do this with the --no-usb option. You won't be able to connect 
to controllers with a USB interface in this case. 

The installation directory of the libraries must be in the library 
search paths when running a MCS2 program.
If they have been installed to their default location, this should be the
case. If they have been installed in a directory that is not in the
search paths, you can append the path to the environment variable 
LD_LIBRARY_PATHS.
    
When you uninstall, all libraries are removed if --no-usb is not set. 
If --no-usb is selected, only the base libraries, not the USB support
libraries are removed. If you have other SmarAct products that need 
some of the installed libraries, you should either not uninstall or
uninstall the MCS2 software and then reinstall the software for the
other products.


Installing the C SDK
--------------------
The --sdk-c option installs the libraries and the C header files to a 
user-definable directory. Documentation and programming example files 
are not installed. Example:

    install_mcs2.sh --license accept --sdk-c ./installdir

If you install to a system path (for example, the default path),
you must execute the installer with sufficient privileges, e.g.

    sudo install_mcs2.sh ...


Installing the Python SDK
-------------------------
The Python SDK requires the installation of the libraries with 
the --libraries option (see above).
Once the libraries have been installed, you may install the Python 
SDK using the Python package installer pip:

    install_mcs2.sh --license accept --libraries
    cd SDK/Python/packages
    pip install smaract.ctl-<version>.zip
    
where <version> must be replaced by the actual version of the
package file.

The uninstall option -u does not remove the installed python package 
from the Python installation. It must be uninstalled manually with
pip

    pip uninstall smaract.ctl-<version>.zip

Please refer to the file doc/Using SmarAct Python SDKs.pdf for
information about installing and using the Python SDK.


Installing Libraries Only
-------------------------
The installer allows to install only the libraries with

    install_mcs2.sh --license accept --libraries

and uninstall them with

    install_mcs2.sh -u --libraries

If --sdk-c is selected, the libraries are automatically installed.  
If --no-usb is selected, the USB support libraries are not installed
or uninstalled.


SYSTEM CONFIGURATION
====================

The information in this section are only relevant if your controller is 
connected via USB to your computer.  

When an MCS or MCS2 controller with a USB interface is connected, it 
is possible that the ftdi_sio driver is automatically loaded for 
that device. In this case the application cannot connect to the 
controller.  
The ftdi_sio driver (kernel module) must be unloaded or blocked from 
loading before launching your application, for example by blacklisting
the ftdi_sio module in /etc/modprobe.d/.

The MCS2 libraries need write access to the USB port the controller
is connected to. In order to set r/w permissions automatically
when the controller is connected, the following udev rules can
be added to /etc/udev/rules.d/

    ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", MODE="666"
    ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", MODE="666"
    ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", MODE="666"

The rule sets the r/w permission for everyone. If this is not
acceptable, adjust the MODE argument.



COMPILING THE C PROGRAMMING EXAMPLES
====================================

If the installation path is /usr (default), a programming example can
be compiled with the following shell commands (assuming the SDK
libraries are installed to /usr):

    cd SDK/C/examples/1_MCS2Example_GetSetProperty
    gcc -o MCS2Example_GetSetProperty MCS2Example_GetSetProperty.c -L/usr/lib -lsmaractctl -I/usr/include

