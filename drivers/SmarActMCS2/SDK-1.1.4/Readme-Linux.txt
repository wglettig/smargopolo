            ================================
                MCS2 Programming Library
                       Linux Notes
                (c) 2019 by SmarAct GmbH
            ================================


REQUIREMENTS
============
This version of the MCS2 SDK requires Linux for x86 (32 bit) or
x86_64 (64 bit) architectures.

The following additional libraries must be installed:
- g++4.9 or later


LICENSE
=======

When installing the software, the SmarAct Software Licensing
Agreement (EULA) must be accepted. By passing "-license accept"
to the installer script, you confirm that you have read and agreed
to the EULA. The EULA text should be included in this software
package. If it is not included please contact SmarAct.


INSTALLATION
============
The library package consists of the shared library libsmaractctl,
C header files and documentation. libsmaractctl requires other
libraries which are included in the package:
libsmaractio, libftd2xx and libftchipid.
All libraries are installed by the 'install_mcs2.sh' script.

The shell script 'install_mcs2.sh' installs the libraries and C
header files. Other files (documentation, programming examples) are
not installed.
The files are installed in the sub-directories lib and include.

    install_mcs2.sh         - installs under /usr
    install_mcs2.sh <path>  - installs under <path>
    install_mcs2.sh -c      - removes previous installations
    (above commands will auto-detect the architecture)

    install_mcs2.sh -x86    - install for x86 architecture
    install_mcs2.sh -x86_64 - install for x86_64 architecture

If you install to a system path (e.g. the default path), you
may need to run 'install_mcs2.sh' with higher access permissions,
for example:
    sudo install_mcs2.sh ...
When you uninstall, all libraries are removed. If you have other SmarAct
products that need some of the other installed libraries you should
either not uninstall or uninstall the MCS2 software and then
reinstall the software for the other products.


SYSTEM CONFIGURATION
====================
When a MCS2 is connected via USB to the computer, it is possible
that the ftdi_sio driver is automatically loaded for that device. In
this case the applications cannot connect to
the MCS2. The kernel modules
  ftdi_sio and usbserial
must be unloaded before launching your application or blocked from
loading, e.g. by blacklisting them in /etc/modprobe.d/.

The MCS2 software needs write access to the USB port and the
USB device under /dev/bus/usb. To automatically set r/w permissions when
a MCS2 is connected, a udev rule can be added to /etc/udev/rules.d/

   ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", MODE="666"

The rule sets the r/w permission for everyone. If this is not
acceptable, adjust the MODE argument.

If your software will communicate with the MCS2 controller
over network only, the above steps are not necessary.
