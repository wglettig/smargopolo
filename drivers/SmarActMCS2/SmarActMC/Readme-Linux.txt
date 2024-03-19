            ================================
            SmarAct Motion Control Software
                       Linux Notes
                (c) 2019 by SmarAct GmbH
            ================================


REQUIREMENTS
============

This version of the SmarAct Motion Control software library requires 
Linux for 32 bit or 64 bit/x86 architecture.

IMPORTANT: the 32 bit libraries are installed by default.
The 64 bit versions are installed if the -x64 option is used. If 
32 bit and 64 bit libraries are installed on the same system, 
they must be installed to different installation paths or they 
will overwrite each other!


INSTALLATION
============
The library package consists of the shared libraries libsmaractmc, 
libmcscontrol, C header files and documentation. The library
requires 3rd party libraries which are also included 
(libftd2xx + libftchipid).
All libraries must be installed on the target computer.

The shell script 'install.sh' installs the libraries and C
header files to a user-definable destination path. Documentation
and other files are NOT installed.
The files are installed in the sub-directories lib and include.
Call:
    install.sh        - to install under /usr
    install.sh <path> - to pass an installation path other than /usr
    install.sh -c     - to remove previous installations
    install.sh -x64   - to install the 64 bit libraries
If you install to a system path (e.g. the default path), you 
must execute install with sufficient privileges, e.g.
    sudo install.sh ...
Note, that when you uninstall, all libraries are removed. If you
have other SmarAct products that need some of the other installed
libraries you should either not uninstall or uninstall the software
and then reinstall the software for the other products.


SYSTEM CONFIGURATION
====================
When an MCS controller is connected to the computer or switched on,
it is possible that the ftdi_sio driver is automatically loaded
for that device. In this case the application cannot connect to
the MCS. The ftdi_sio driver (kernel module) must be unloaded before
launching your application or blocked from loading, e.g. by 
blacklisting the ftdi_sio module in /etc/modprobe.d/.

The MCS Control library needs write access to the USB port the MCS 
device is connected to. To automatically set r/w permissions when 
an MCS is connected, the following udev rules can be added 
to /etc/udev/rules.d/

   ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", MODE="666"
   ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", MODE="666"
   ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", MODE="666"

The rule sets the r/w permission for everyone. If this is not
acceptable, adjust the MODE argument.


COMPILING THE SAMPLES
=====================
If the installation path is /usr (default), a sample program can
be compiled with the following shell commands:
cd samples
gcc -o program sourcecode.c -L/usr/lib -lsmaractmc -I/usr/include

