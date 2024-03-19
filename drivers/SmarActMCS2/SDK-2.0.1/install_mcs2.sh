#!/bin/sh
# install script for the SmarAct MCS2 SDK for Linux
# (c) 2020 SmarAct GmbH


showhelp()
{
  printf "Usage:\n\
  $(basename $0) -h\n\
  $(basename $0) [OPTION]... [PATH]\n\
Options:\n\
  -h            Display a short help text and quit.\n\
  -u            Uninstall software.\n\
  --version     Display version and quit.\n\
  --license v   With '--license accept' you confirm that you have read\n\
                the SmarAct Software License Agreement (EULA) and accept it.\n\
                It must be accepted in order to install and use the
                software.\n\
                The EULA text is included in the software package.\n\
                If it is not included, please contact SmarAct.\n\
  --libraries   Install the MCS2 libraries only\n\
  --sdk-c       Install the C SDK and libraries\n\
  --no-usb      Suppresses the installation of the USB support libraries\n\
                which are installed by default\n\
  --arch a      (optional) The target architecture a: x86 or x86_64.\n\
                The system architecture is used if omitted.\n\
  PATH          (optional) The base directory for installing,\n\
                e.g. /usr or /opt. Defaults to /usr if omitted.\n" \
>&2
}


print_version()
{
  VINFO=$(cat ${SRCPATH}/version-*)
  echo "${VINFO}"
}

uninstall_libs()
{
  echo "Removing MCS2 Libraries ($ARCH) from $IPATH"
  rm -f "$IPATH/lib"/libsmaractctl*
  rm -f "$IPATH/lib"/libsmaractio*

  if [ ! "$NO_USB" ]; then
    rm -f "$IPATH/lib"/libftd2xx.so*
    rm -f "$IPATH/lib"/libftchipid*
  fi
}


install_libs()
{
  if [ "$LIBS_INSTALLED" -eq "1" ]; then
    return
  fi

  uninstall_libs

  echo "Installing MCS2 Libraries ($ARCH) to $IPATH"
  LIBS_INSTALLED=1

  cp -a -f "$REDIST/arch_$ARCH/lib"/libsmaractctl.so* "$IPATH/lib/"
  cp -a -f "$REDIST/arch_$ARCH/lib"/libsmaractio.so* "$IPATH/lib/"

  if [ ! "$NO_USB" ]; then
    cp -a -f "$REDIST/arch_$ARCH/lib"/libftd2xx.so* "$IPATH/lib/"
    cp -a -f "$REDIST/arch_$ARCH/lib"/libftchipid.so* "$IPATH/lib/"
  fi

  ldconfig -n "$IPATH/lib"
}


install_sdk_c()
{
  echo "Installing MCS2 C SDK ($ARCH) to $IPATH"

  if [ ! -d "$IPATH/lib" ]; then
    mkdir -p "$IPATH/lib"
  fi
  if [ ! -d "$IPATH/include" ]; then
    mkdir -p "$IPATH/include"
  fi
  cp -a -f "$SDKC/include"/*.h "$IPATH/include/"

  install_libs

  cd $OLDP
}


do_install()
{
  if [ "$LICENSE_ACCEPTED" != "1" ]; then
    echo "*******************************************************"
    echo "Could not install:\n\
In order to install this software, the license agreement\n\
must be accepted. Please read the installer help."
    echo "*******************************************************"
    exit 1
  fi

  if [ ! "$SELECT_SDK_C" ] && [ ! "$SELECT_LIBS" ]; then
    echo "*******************************************************"
    echo "Could not install:\n\
No installation option is selected.\n\
There is nothing to install."
    echo "*******************************************************"
    exit 1
  fi

  print_version

  if [ "$SELECT_SDK_C" ]; then
    install_sdk_c
  fi
  if [ "$SELECT_LIBS" ]; then
    install_libs
  fi
}


do_uninstall()
{
  echo "Uninstalling MCS2 software from $IPATH"
  uninstall_libs
  rm -f "$IPATH/include/SmarActControl.h"
  rm -f "$IPATH/include/SmarActControlConstants.h"
}


get_arch()
{
    local A=$(uname -m)
    if [ "$A" = "x86_64" ]; then
        echo "x86_64"
    elif [ "$A" = "i386" ] || [ "$A" = "i686" ]; then
        echo "x86"
    fi
    # else: empty
}


read_options()
{
    while [ $# -gt 0 ]
    do
        case "$1" in
            -u) DO_UNINSTALL=1
                ;;
            --version) print_version
              exit 0
              ;;
            --license) LICENSE_VALUE="$2"
                shift
                ;;
            --arch) ARCH="$2"
                shift
                ;;
            --sdk-c) SELECT_SDK_C=1
                ;;
            --libraries) SELECT_LIBS=1
                ;;
            --no-usb) NO_USB=1
                ;;
             *) IPATH="$1"
                ;;
        esac
        shift
    done
}




self="${0#./}"
base="${self%/*}"

SRCPATH=""
if [ "$base" = "$self" ]; then
    SRCPATH="$(pwd)"
else
    SRCPATH="$(pwd)/$base"
fi ;

SDKPATH=${SRCPATH}/SDK
SDKC=${SDKPATH}/C
SDKPY=${SDKPATH}/Python
REDIST=${SDKPATH}/Redistributable

LIBS_INSTALLED=0

# may be empty if trying to install on non-x86 architecture
SYS_ARCH=$(get_arch)


DO_UNINSTALL=0
IPATH="/usr"
LICENSE_VALUE="noaccept"
ARCH=$SYS_ARCH
LICENSE_ACCEPTED=0

if [ "$1" = "-h" ]; then
  showhelp;
  exit 2
fi


read_options $*

if [ "$ARCH" = "" ]; then
  echo "Unknown or unsupported architecture."
  return 1
fi

if [ ! -d "$IPATH" ]; then
  echo "Installation path does not exist: $IPATH"
  return 1
fi


if [ "$LICENSE_VALUE" = "accept" ]; then
  LICENSE_ACCEPTED=1
fi


if [ "$DO_UNINSTALL" = "1" ]; then
  do_uninstall
else
  do_install
fi
