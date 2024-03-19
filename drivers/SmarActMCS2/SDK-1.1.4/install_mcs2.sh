#!/bin/sh
# install script for MCS2 SDK for Linux


showhelp()
{
  printf "Usage:\n\
  $(basename $0) -h\n\
  $(basename $0) [-u] [-license v] [-arch a] [path]\n\
Options:\n\
  -u          Uninstall existing libraries from path.\n\
  -license v  With '-license accept' you confirm that you have read\n\
              and accept the SmarAct Software License Agreement (EULA).\n\
              It must be accepted in order to install and use the
              software.\n\
              The EULA text should be included in the software package.\n\
              If it is not included, please contact SmarAct.\n\
  -arch a     (optional) The target architecture a: x86 or x86_64.\n\
              The system architecture is used if omitted\n\
  path        (optional) The base directory for installing,\n\
              e.g. /usr or /opt. Defaults to /usr if omitted.\n" \
>&2
}


uninstall_sdk()
{
  echo "Uninstalling MCS2 software from $IPATH..."
  rm -f "$IPATH"/lib/libsmaractio*
  rm -f "$IPATH"/lib/libsmaractctl*
  rm -f "$IPATH"/lib/libftd2xx*
  rm -f "$IPATH"/lib/libftchipid*
  rm -f "$IPATH"/include/SmarActControl.h
  rm -f "$IPATH"/include/SmarActControlConstants.h
}


install_sdk()
{
  if [ "$LICENSE_ACCEPTED" != "1" ]; then
    echo "*******************************************************"
    echo "Could not install:\n\
In order to install this software, the license agreement\n\
must be accepted. Please read the installer help."
    echo "*******************************************************"
    exit 1
  fi
  
  echo "Installing MCS2 SDK ($ARCH) for Linux to $IPATH..."
  
  if [ ! -d "$IPATH/lib" ]; then
    mkdir -p "$IPATH/lib"
  fi
  if [ ! -d "$IPATH/include" ]; then
    mkdir -p "$IPATH/include"
  fi
  cp -a -f "$SRCPATH/include"/*.h "$IPATH/include/"
    
  rm -f "$IPATH/lib"/libsmaractio.so.1*
  rm -f "$IPATH/lib"/libsmaractctl.so.1*
  rm -f "$IPATH/lib"/libftd2xx.so.1*
    
  cp -a -f "$SRCPATH/arch_$ARCH/lib"/libsmaractio.so* "$IPATH/lib/"
  cp -a -f "$SRCPATH/arch_$ARCH/lib"/libsmaractctl.so* "$IPATH/lib/"
  cp -a -f "$SRCPATH/arch_$ARCH/lib"/libftd2xx.so* "$IPATH/lib/"
  cp -a -f "$SRCPATH/arch_$ARCH/lib"/libftchipid.so* "$IPATH/lib/"
  ldconfig -n "$IPATH/lib"

  cd $OLDP
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
            -license) LICENSE_VALUE="$2"
                shift
                ;;
            -arch) ARCH="$2"
                shift
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
  uninstall_sdk;
else
  install_sdk;
fi



