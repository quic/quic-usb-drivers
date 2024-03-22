#!/bin/bash

# Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause

DEST_QUIC_PATH=/opt/QUIC/
DEST_USB_PATH=/opt/QUIC/USB
DEST_INS_RMNET_PATH=/opt/QUIC/USB/rmnet
DEST_INS_QDSS_PATH=/opt/QUIC/USB/diag
DEST_INF_PATH=/opt/QUIC/USB/diag/InfParser
DEST_QDSS_DAIG_PATH=/opt/QUIC/USB/diag/QdssDiag
DEST_SIGN_PATH=/opt/QUIC/sign
QUIC_MODBIN_DIR=/sbin
QUIC_MAKE_DIR=/usr/bin
QUIC_MODULE_INF_NAME=qtiDevInf.ko
QUIC_MODULE_QDSS_DIAG_NAME=QdssDiag.ko
QUIC_MODULE_RMNET_NAME=GobiNet.ko
QUIC_DIAG_INF_PATH=/opt/QUIC/USB/diag/qtiser.inf
QUIC_QDSS_INF_PATH=/opt/QUIC/USB/diag/qdbusb.inf
QUIC_MODEM_INF_PATH=/opt/QUIC/USB/serial/qtimdm.inf
DEST_INS_SERIAL_PATH=/opt/QUIC/USB/serial
QUIC_UDEV_PATH=/etc/udev/rules.d
QUIC_MODULE_GOBISERIAL_NAME=GobiSerial.ko
QUIC_SERIAL=/lib/modules/`uname -r`/kernel/drivers/usb/serial
QUIC_QMI_WWAN=/lib/modules/`uname -r`/kernel/drivers/net/usb
QUIC_NET=/lib/modules/`uname -r`/kernel/drivers/net
QUIC_LN_RM_MK_DIR=/bin
MODULE_BLACKLIST=/etc/modprobe.d
OS_RELEASE="`cat /etc/os-release | grep PRETTY_NAME`"
OSName=`echo $OS_RELEASE | awk -F= '{printf $2}'`
KERNEL_VERSION=`uname -r`

#check and install mokutil package
if [ ! -f "$QUIC_MAKE_DIR/mokutil" ]; then
   echo "Error: mokutil not found, installing..\n"

   if [[ $OSName =~ "Red Hat Enterprise Linux" ]]; then
      sudo dnf install -y mokutil
      sudo dnf install -y keyutils
   fi

   if [[ $OSName =~ "Ubuntu" ]]; then
      sudo apt-get install -y mokutil
      sudo apt-get install -y keyutils
   fi
fi

QUIC_SECURE_BOOT_CHECK=`mokutil --sb-state`

if [[ $QUIC_SECURE_BOOT_CHECK = "SecureBoot enabled" ]]; then
   QUIC_PUBLIC_KEY_VERIFY=`mokutil --test-key $DEST_SIGN_PATH/Signkey_pub.der`
fi

if [  ! -d $DEST_USB_PATH  ]; then
   echo "Error: $DEST_USB_PATH doesn't exist. Creating Now."
   $QUIC_LN_RM_MK_DIR/mkdir -m 0755 -p $DEST_USB_PATH
fi

if [ $# == 0 ]; then
   echo "Usage: QUICDevInstaller.sh <install | uninstall>"
   exit 1
else
   if [ $1 == "uninstall" ]; then
      echo "Operating System: $OSName"
      echo "Kernel Version: "\""`uname -r`"\"""

      if [ -d $DEST_INS_QDSS_PATH ]; then
         $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
         if [ ! -d $DEST_INS_QDSS_PATH ]; then
            echo "Successfully removed $DEST_INS_QDSS_PATH"
         else
            echo "Failed to remove $DEST_INS_QDSS_PATH"
         fi
      else
         echo "$DEST_INS_QDSS_PATH does not exist, nothing to remove"
      fi
      if [ -d $DEST_INS_RMNET_PATH ]; then
         $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
         if [ ! -d $DEST_INS_RMNET_PATH ]; then
            echo "Successfully removed $DEST_INS_RMNET_PATH"
         else
            echo "Failed to remove $DEST_INS_RMNET_PATH"
         fi
      else
         echo "$DEST_INS_RMNET_PATH does not exist, nothing to remove"
      fi
      if [ -d $DEST_INS_SERIAL_PATH ]; then
         $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_SERIAL_PATH
         if [ ! -d $DEST_INS_SERIAL_PATH ]; then
            echo "Successfully removed $DEST_INS_SERIAL_PATH"
         else
            echo "Failed to remove $DEST_INS_SERIAL_PATH"
         fi
      else
         echo "$DEST_INS_SERIAL_PATH does not exist, nothing to remove"
      fi

      if [ -f $QUIC_UDEV_PATH/qti_usb_device.rules ]; then
         rm -r $QUIC_UDEV_PATH/qti_usb_device.rules
         if [ ! -f $QUIC_UDEV_PATH/qti_usb_device.rules ]; then
            echo "Successfully removed $QUIC_UDEV_PATH/qti_usb_device.rules"
         else
            echo "Failed to remove $QUIC_UDEV_PATH/qti_usb_device.rules"
         fi
      else
         echo "$QUIC_UDEV_PATH/qti_usb_device.rules does not exist, nothing to remove"
      fi
      # < "Redundant 80-net-setup-link.rules. This code will be eliminated after few releases"
      if [ -f $QUIC_UDEV_PATH/80-net-setup-link.rules ]; then
         sed -i '/GobiQMI/d' $QUIC_UDEV_PATH/80-net-setup-link.rules
         if [ ! -s $QUIC_UDEV_PATH/80-net-setup-link.rules ]; then
            echo "Removed GobiQMI rule from $QUIC_UDEV_PATH/80-net-setup-link.rules"
            rm -rf $QUIC_UDEV_PATH/80-net-setup-link.rules
            if [ ! -f $QUIC_UDEV_PATH/80-net-setup-link.rules ]; then
               echo "File was empty. Removed $QUIC_UDEV_PATH/80-net-setup-link.rules successfully"
            else
               echo "File is empty but Failed to remove $QUIC_UDEV_PATH/80-net-setup-link.rules"
            fi
         else
               echo "Pre-Existing data $QUIC_UDEV_PATH/80-net-setup-link.rules, so not removing it."
         fi
      else
         echo "$QUIC_UDEV_PATH/80-net-setup-link.rules does not exist, nothing to remove"
      fi
      # >
      if [ -f $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules ]; then
         sed -i '/usb/d' $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules
         if [ ! -s $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules ]; then
            echo "Removed GobiQMI rule from $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules"
            rm -rf $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules
            if [ ! -f $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules ]; then
               echo "File was empty. Removed $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules successfully"
            else
               echo "File is empty but Failed to remove $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules"
            fi
         else
               echo "Pre-Existing data $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules, so not removing it."
         fi
      else
         echo "$QUIC_UDEV_PATH/80-gobinet-usbdevice.rules does not exist, nothing to remove"
      fi

      # Informs udev deamon to reload the newly added device rule and re-trigger service
      sudo udevadm control --reload-rules
      sudo udevadm trigger

      if [ "`lsmod | grep GobiSerial`" ]; then
         ($QUIC_MODBIN_DIR/rmmod $QUIC_MODULE_GOBISERIAL_NAME && echo "$QUIC_MODULE_GOBISERIAL_NAME removed successfully") || { echo "$QUIC_MODULE_GOBISERIAL_NAME in use"; echo "Note: Close all applications that make use of the driver, including QUTS clients."; echo "ps -aux | grep QUTS, sudo kill -9 <PID>"; echo "Try $1ation again!"; exit 1; }
      else
         echo "Module $QUIC_MODULE_GOBISERIAL_NAME is not currently loaded"
      fi
      if [ "`lsmod | grep GobiNet`" ]; then
         ( $QUIC_MODBIN_DIR/rmmod $QUIC_MODULE_RMNET_NAME && echo "$QUIC_MODULE_RMNET_NAME removed successfully" ) || { echo "$QUIC_MODULE_RMNET_NAME in use"; echo "Note: Close all applications that make use of the driver, including QUTS clients."; echo "ps -aux | grep QUTS, sudo kill -9 <PID>"; echo "Try $1ation again!"; exit 1; }
      else
         echo "Module $QUIC_MODULE_RMNET_NAME is not currently loaded"
      fi
      if [ "`lsmod | grep QdssDiag`" ]; then
         ($QUIC_MODBIN_DIR/rmmod $QUIC_MODULE_QDSS_DIAG_NAME && echo "$QUIC_MODULE_QDSS_DIAG_NAME removed successfully") || { echo "$QUIC_MODULE_QDSS_DIAG_NAME in use"; echo "Note: Close all applications that make use of the driver, including QUTS clients."; echo "ps -aux | grep QUTS, sudo kill -9 <PID>"; echo "Try $1ation again!"; exit 1; }
      else
         echo "Module $QUIC_MODULE_QDSS_DIAG_NAME is not currently loaded"
      fi
      if [ "`lsmod | grep qtiDevInf`" ]; then
         ($QUIC_MODBIN_DIR/rmmod $QUIC_MODULE_INF_NAME && echo "$QUIC_MODULE_INF_NAME removed successfully") || { echo "$QUIC_MODULE_INF_NAME in use"; echo "Note: Close all applications that make use of the driver, including QUTS clients."; echo "ps -aux | grep QUTS, sudo kill -9 <PID>"; echo "Try $1ation again!"; exit 1; }
      else
         echo "Module $QUIC_MODULE_INF_NAME is not currently loaded"
      fi

      MODLOADED="`/sbin/lsmod | grep usb_wwan`"
      if [ "$MODLOADED" != "" ]; then
         echo "usb_wwan module is already loaded. nothing to do"
      elif [  -f $QUIC_SERIAL/usb_wwan_dup.ko ]; then
         echo "usb_wwan_dup is found. restoring to usb_wwan"
         mv /lib/modules/`uname -r`/kernel/drivers/usb/serial/usb_wwan_dup.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/usb_wwan.ko
         $QUIC_MODBIN_DIR/insmod /lib/modules/`uname -r`/kernel/drivers/usb/serial/usb_wwan.ko

         MODLOADED="`/sbin/lsmod | grep usb_wwan`"
         if [ "$MODLOADED" != "" ]; then
            echo "Successfully loaded usb_wwan module."
         fi
      fi

      MODLOADED="`/sbin/lsmod | grep qcserial`"
      if [ "$MODLOADED" != "" ]; then
         echo "qcserial module is already loaded. nothing to do"
      elif [  -f $QUIC_SERIAL/qcserial_dup.ko ]; then
         echo "qcserial_dup is found. restoring to qcserial"
         mv /lib/modules/`uname -r`/kernel/drivers/usb/serial/qcserial_dup.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/qcserial.ko
         $QUIC_MODBIN_DIR/insmod /lib/modules/`uname -r`/kernel/drivers/usb/serial/qcserial.ko

         MODLOADED="`/sbin/lsmod | grep qcserial`"
         if [ "$MODLOADED" != "" ]; then
            echo "Successfully loaded qcserial module."
         fi
      fi

      MODLOADED="`/sbin/lsmod | grep option`"
      if [ "$MODLOADED" != "" ]; then
         echo "option module is already loaded. nothing to do"
      elif [  -f $QUIC_SERIAL/option_dup.ko ]; then
         echo "option_dup is found. restoring to option"
         mv /lib/modules/`uname -r`/kernel/drivers/usb/serial/option_dup.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/option.ko
         $QUIC_MODBIN_DIR/insmod /lib/modules/`uname -r`/kernel/drivers/usb/serial/option.ko

         MODLOADED="`/sbin/lsmod | grep option`"
         if [ "$MODLOADED" != "" ]; then
            echo "Successfully loaded option module."
         fi
      fi

      MODLOADED="`/sbin/lsmod | grep qmi_wwan`"
      if [ "$MODLOADED" != "" ]; then
         echo "qmi_wwan module is already loaded. nothing to do"
      elif [  -f $QUIC_QMI_WWAN/qmi_wwan_dup.ko ]; then
         echo "qmi_wwan_dup is found. restoring to qmi_wwan"
         mv /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-wdm_dup.ko /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-wdm.ko
         mv /lib/modules/`uname -r`/kernel/drivers/net/usb/qmi_wwan_dup.ko /lib/modules/`uname -r`/kernel/drivers/net/usb/qmi_wwan.ko
         $QUIC_MODBIN_DIR/insmod /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-wdm.ko
         $QUIC_MODBIN_DIR/insmod /lib/modules/`uname -r`/kernel/drivers/net/usb/qmi_wwan.ko

         MODLOADED="`/sbin/lsmod | grep qmi_wwan`"
         if [ "$MODLOADED" != "" ]; then
            echo "Successfully loaded qmi_wwan module."
         fi
      fi

      if [ "`grep -nr 'Qualcomm clients' /etc/modprobe.d/blacklist.conf`" != "" ]; then
         sed -i '/# Blacklist these module so that Qualcomm clients use only/d' /etc/modprobe.d/blacklist.conf
         sed -i '/# GobiNet, GobiSerial, QdssDiag, qtiDevInf driver/d' /etc/modprobe.d/blacklist.conf
      fi

      MOD_EXIST="`grep -nr  'blacklist qcserial' /etc/modprobe.d/blacklist.conf`"
      if [ "$MOD_EXIST" != "" ]; then
         sed -i '/qcserial/d' $MODULE_BLACKLIST/blacklist.conf
         echo "Successfully removed qcserial from $MODULE_BLACKLIST/blacklist.conf"
      fi

      MOD_EXIST="`grep -nr  'blacklist qmi_wwan' /etc/modprobe.d/blacklist.conf`"
      if [ "$MOD_EXIST" != "" ]; then
         sed -i '/qmi_wwan/d' $MODULE_BLACKLIST/blacklist.conf
         echo "Successfully removed qmi_wwan from $MODULE_BLACKLIST/blacklist.conf"
      fi

      MOD_EXIST="`grep -nr  'blacklist option' /etc/modprobe.d/blacklist.conf`"
      if [ "$MOD_EXIST" != "" ]; then
         sed -i '/option/d' $MODULE_BLACKLIST/blacklist.conf
         echo "Successfully removed option from $MODULE_BLACKLIST/blacklist.conf"
      fi

      MOD_EXIST="`grep -nr  'blacklist usb_wwan' /etc/modprobe.d/blacklist.conf`"
      if [ "$MOD_EXIST" != "" ]; then
         sed -i '/usb_wwan/d' $MODULE_BLACKLIST/blacklist.conf
         echo "Successfully removed usb_wwan from $MODULE_BLACKLIST/blacklist.conf"
      fi

      #change to permission to default mode
      $QUIC_LN_RM_MK_DIR/chmod 644 $MODULE_BLACKLIST/blacklist.conf

      echo "Removing modules for /etc/modules."
      MODUPDATE="`grep -r qtiDevInf /etc/modules`"
      if [ "$MODUPDATE" == "qtiDevInf" ]; then
	  sed -i '/qtiDevInf/d' /etc/modules
      fi
      MODUPDATE="`grep -r QdssDiag /etc/modules`"
      if [ "$MODUPDATE" == "QdssDiag" ]; then
	  sed -i '/QdssDiag/d' /etc/modules
      fi
      MODUPDATE="`grep -r GobiNet /etc/modules`"
      if [ "$MODUPDATE" == "GobiNet" ]; then
	  sed -i '/GobiNet/d' /etc/modules
      fi
      MODUPDATE="`grep -r GobiSerial /etc/modules`"
      if [ "$MODUPDATE" == "GobiSerial" ]; then
	  sed -i '/GobiSerial/d' /etc/modules
      fi
      if [[ $OSName != *"Red Hat Enterprise Linux"* ]]; then
      	MODUPDATE="`grep -nr  'iface usb0 inet static' /etc/network/interfaces`"
      	if [ "$MODUPDATE" != "" ]; then
        	 sed -i '/iface usb0 inet static/d' /etc/network/interfaces
      	fi
      fi

      echo "Removing modules from $QUIC_SERIAL"
      if [ -f $QUIC_SERIAL/$QUIC_MODULE_GOBISERIAL_NAME ]; then
         rm -rf $QUIC_SERIAL/$QUIC_MODULE_GOBISERIAL_NAME
      fi
      echo "Removing modules from $QUIC_QMI_WWAN"
      if [ -f $QUIC_QMI_WWAN/$QUIC_MODULE_QDSS_DIAG_NAME ]; then
         rm -rf $QUIC_QMI_WWAN/$QUIC_MODULE_QDSS_DIAG_NAME
      fi
      if [ -f $QUIC_QMI_WWAN/$QUIC_MODULE_RMNET_NAME ]; then
         rm -rf $QUIC_QMI_WWAN/$QUIC_MODULE_RMNET_NAME
      fi
      if [ -f $QUIC_QMI_WWAN/$QUIC_MODULE_INF_NAME ]; then
         rm -rf $QUIC_QMI_WWAN/$QUIC_MODULE_INF_NAME
      fi
	   
      depmod

      # if [ -f $DEST_USB_PATH/ReleaseNotes*.txt ]; then
      #    $QUIC_LN_RM_MK_DIR/rm -rf $DEST_USB_PATH/ReleaseNotes*.txt
      # fi

	   echo "Uninstallation completed successfully."
      exit 0
   else
      if [ $1 != "install" ]; then
         echo "Usage: QUICDevInstaller.sh <install | uninstall>"
         exit 1
      fi
   fi
fi

######## Installation ###########

if [[ $OSName =~ "Ubuntu" ]]; then
   sudo apt-get update
   sudo apt-get install -y build-essential
   sudo apt-get install -y gawk
   sudo apt-get install -y python3-tk
fi

IFS=. read -r major_ver minor_ver patch_ver <<< "$KERNEL_VERSION"
if [[ $OSName =~ "Ubuntu 22." ]] && (( "$major_ver" >= 6 && "$minor_ver" >= 5 )); then
   echo "Installing gcc 12 version ..."
   sudo apt install -y gcc-12 g++-12
fi

echo "======================================================================================="
echo "======================================================================================="
echo " "

echo "Operating System: $OSName"
echo "Kernel Version: "\"$KERNEL_VERSION\"""

if [ -f ./version.h ]; then
   VERSION="`grep -r '#define DRIVER_VERSION' version.h`"
   DRIVER_VERSION=`echo $VERSION | awk '{printf $3}'`
   echo "Driver Version: $DRIVER_VERSION"
fi

echo "Installing at the following paths:"
echo $DEST_INS_SERIAL_PATH
echo $DEST_INS_QDSS_PATH
echo $DEST_INS_RMNET_PATH

# this script will use in qik uninstallation process
if [ -f "$DEST_USB_PATH/QcDevDriver.sh" ]; then
   echo "Delete and copy again (QcDevDriver.sh)"
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_USB_PATH/QcDevDriver.sh
   $QUIC_LN_RM_MK_DIR/cp -rf ./QcDevDriver.sh $DEST_USB_PATH/
else
   echo "Does not exist and copying now.. (QcDevDriver.sh)"
   $QUIC_LN_RM_MK_DIR/cp -rf ./QcDevDriver.sh $DEST_USB_PATH/
fi

# Create directories
if [ -d $DEST_INS_SERIAL_PATH ]; then
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_SERIAL_PATH
fi

$QUIC_LN_RM_MK_DIR/mkdir -m 0755 -p $DEST_INS_SERIAL_PATH
if [  ! -d $DEST_INS_SERIAL_PATH  ]; then
   echo "Error: Failed to create installation path, please run installer under root."
   exit 1
fi

if [ -d $DEST_INS_QDSS_PATH ]; then
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
fi

$QUIC_LN_RM_MK_DIR/mkdir -m 0755  -p $DEST_INS_QDSS_PATH
if [  ! -d $DEST_INS_QDSS_PATH  ]; then
   echo "Error: Failed to create installation path, please run installer under root."
   exit 1
fi

if [ -d $DEST_INF_PATH ]; then
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INF_PATH
fi

$QUIC_LN_RM_MK_DIR/mkdir -m 0755  -p $DEST_INF_PATH
if [  ! -d $DEST_INF_PATH  ]; then
   echo "Error: Failed to create installation path, please run installer under root."
   exit 1
fi

if [ -d $DEST_QDSS_DAIG_PATH ]; then
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_QDSS_DAIG_PATH
fi

$QUIC_LN_RM_MK_DIR/mkdir -m 0755  -p $DEST_QDSS_DAIG_PATH
if [  ! -d $DEST_QDSS_DAIG_PATH  ]; then
   echo "Error: Failed to create installation path, please run installer under root."
   exit 1
fi

# Important: Do not delete or recreate the "sign" folder.
# The sign files will be automatically generated whenever the Signpub key is not enrolled.
$QUIC_LN_RM_MK_DIR/mkdir -m 0777  -p $DEST_SIGN_PATH
if [ ! -d $DEST_SIGN_PATH ]; then
   echo "Error: Failed to create installation path, please run installer under root."
   exit 1
fi

# $QUIC_LN_RM_MK_DIR/cp ./GobiSerial/GobiSerial.c $DEST_INS_SERIAL_PATH
# if [ ! -f $DEST_INS_SERIAL_PATH/GobiSerial.c ]; then
#    echo "Error: Failed to copy GobiSerial.c to installation path, installation abort."
#    rm -r $DEST_INS_SERIAL_PATH
#    exit 1
# fi

# $QUIC_LN_RM_MK_DIR/cp ./GobiSerial/GobiSerial.h $DEST_INS_SERIAL_PATH
# if [ ! -f $DEST_INS_SERIAL_PATH/GobiSerial.h ]; then
#    echo "Error: Failed to copy GobiSerial.h to installation path, installation abort."
#    rm -r $DEST_INS_SERIAL_PATH
#    exit 1
# fi

# $QUIC_LN_RM_MK_DIR/cp ./GobiSerial/Makefile  $DEST_INS_SERIAL_PATH
# if [ ! -f $DEST_INS_SERIAL_PATH/Makefile ]; then
#    echo "Error: Failed to copy Makefile installation path, installation abort."
#    rm -r $DEST_INS_SERIAL_PATH
#    exit 1
# fi

# $QUIC_LN_RM_MK_DIR/cp ./GobiSerial/qtidev.pl  $DEST_INS_SERIAL_PATH
# if [ ! -f $DEST_INS_SERIAL_PATH/qtidev.pl ]; then
#    echo "Error: Failed to copy qtidev.pl installation path, installation abort."
#    rm -r $DEST_INS_SERIAL_PATH
#    exit 1
# fi
# $QUIC_LN_RM_MK_DIR/chmod 755 $DEST_INS_SERIAL_PATH/qtidev.pl

$QUIC_LN_RM_MK_DIR/cp ./GobiSerial/qtiname.inf $DEST_INS_SERIAL_PATH
if [ ! -f $DEST_INS_SERIAL_PATH/qtiname.inf ]; then
   echo "Error: Failed to copy qtiname.inf installation path, installation abort."
   rm -r $DEST_INS_SERIAL_PATH
   exit 1
fi
$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_INS_SERIAL_PATH/qtiname.inf

$QUIC_LN_RM_MK_DIR/cp ./GobiSerial/qtimdm.inf $DEST_INS_SERIAL_PATH
if [ ! -f $DEST_INS_SERIAL_PATH/qtimdm.inf ]; then
   echo "Error: Failed to copy qtiname.inf installation path, installation abort."
   rm -r $DEST_INS_SERIAL_PATH
   exit 1
fi
$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_INS_SERIAL_PATH/qtimdm.inf


$QUIC_LN_RM_MK_DIR/cp ./GobiSerial/qtiname.inf $DEST_INS_QDSS_PATH/
if [ ! -f $DEST_INS_QDSS_PATH//qtiname.inf ]; then
   echo "Error: Failed to copy qtiname.inf installation path, installation abort."
   rm -r $DEST_INS_QDSS_PATH/
   exit 1
fi
$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_INS_SERIAL_PATH/qtiname.inf

$QUIC_LN_RM_MK_DIR/cp ./GobiSerial/qtimdm.inf $DEST_INS_QDSS_PATH/
if [ ! -f $DEST_INS_QDSS_PATH//qtimdm.inf ]; then
   echo "Error: Failed to copy qtiname.inf installation path, installation abort."
   rm -r $DEST_INS_QDSS_PATH/
   exit 1
fi
$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_INS_QDSS_PATH/qtimdm.inf


$QUIC_LN_RM_MK_DIR/cp ./QdssDiag/qdbusb.inf $DEST_INS_QDSS_PATH/
if [ ! -f $DEST_INS_QDSS_PATH/qdbusb.inf ]; then
   echo "Error: Failed to copy 'qdbusb.inf' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./QdssDiag/qtiser.inf $DEST_INS_QDSS_PATH/
if [ ! -f $DEST_INS_QDSS_PATH/qtiser.inf ]; then
   echo "Error: Failed to copy 'qtiser.inf' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_INS_QDSS_PATH/qdbusb.inf
$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_INS_QDSS_PATH/qtiser.inf

$QUIC_LN_RM_MK_DIR/cp ./InfParser/qtiDevInf.h $DEST_INF_PATH/
if [ ! -f $DEST_INF_PATH/qtiDevInf.h ]; then
   echo "Error: Failed to copy 'InfParser/qtiDevInf.h' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./InfParser/qtiDevInf.c $DEST_INF_PATH/
if [ ! -f $DEST_INF_PATH/qtiDevInf.c ]; then
   echo "Error: Failed to copy 'InfParser/qtiDevInf.c' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./InfParser/Makefile $DEST_INF_PATH/
if [ ! -f $DEST_INF_PATH/Makefile ]; then
   echo "Error: Failed to copy 'InfParser/Makefile' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./QdssDiag/qtiDevInf.h $DEST_QDSS_DAIG_PATH/
if [ ! -f $DEST_QDSS_DAIG_PATH/qtiDevInf.h ]; then
   echo "Error: Failed to copy 'QdssDiag/qtiDevInf.h' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./QdssDiag/qtiDiag.h $DEST_QDSS_DAIG_PATH/
if [ ! -f $DEST_QDSS_DAIG_PATH/qtiDiag.h ]; then
   echo "Error: Failed to copy 'QdssDiag/qtiDiag.h' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./QdssDiag/qtiDiag.c $DEST_QDSS_DAIG_PATH/
if [ ! -f $DEST_QDSS_DAIG_PATH/qtiDiag.c ]; then
   echo "Error: Failed to copy 'QdssDiag/qtiDiag.c' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./Makefile $DEST_QDSS_DAIG_PATH/
if [ ! -f $DEST_QDSS_DAIG_PATH/Makefile ]; then
   echo "Error: Failed to copy 'QdssDiag/Makefile' to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   exit 1
fi

if [ -d $DEST_INS_RMNET_PATH ]; then
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
fi

$QUIC_LN_RM_MK_DIR/mkdir -p $DEST_INS_RMNET_PATH
if [  ! -d $DEST_INS_RMNET_PATH  ]; then
   echo "Error: Failed to create installation path, please run installer under root."
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/IPAssignmentScript.sh $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/IPAssignmentScript.sh ]; then
   echo "Error: Failed to copy IPAssignmentScript.sh to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/chmod 755 $DEST_INS_RMNET_PATH/IPAssignmentScript.sh

$QUIC_LN_RM_MK_DIR/cp ./rmnet/GobiUSBNet.c $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/GobiUSBNet.c ]; then
   echo "Error: Failed to copy GobiUSBNet.c to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/QMIDevice.c $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/QMIDevice.c ]; then
   echo "Error: Failed to copy QMIDevice.c to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/QMIDevice.h $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/QMIDevice.h ]; then
   echo "Error: Failed to copy QMIDevice.h to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/QMI.c $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/QMI.c ]; then
   echo "Error: Failed to copy QMI.c to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/QMI.h $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/QMI.h ]; then
   echo "Error: Failed to copy QMI.h to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/qmap.c $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/qmap.c ]; then
   echo "Error: Failed to copy qmap.c to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/qmap.h $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/qmap.h ]; then
   echo "Error: Failed to copy qmap.h to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/Structs.h $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/Structs.h ]; then
   echo "Error: Failed to copy Structs.h to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/Makefile  $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/Makefile ]; then
   echo "Error: Failed to copy Makefile installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/qtiwwan.inf $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/qtiwwan.inf ]; then
   echo "Error: Failed to copy qtiwwan.inf installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
   exit 1
fi
$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_INS_RMNET_PATH/qtiwwan.inf

#DEST_SIGN_PATH=/opt/QUIC/sign/
$QUIC_LN_RM_MK_DIR/cp ./sign/SignConf.config $DEST_SIGN_PATH
$QUIC_LN_RM_MK_DIR/chmod 777 $DEST_SIGN_PATH/SignConf.config
awk -i inplace -v name=`hostname` '{gsub(/O = /,"O = "name)}1' $DEST_SIGN_PATH/SignConf.config
awk -i inplace -v name=`hostname` '{gsub(/CN = /,"CN = "name" Signing Key")}1' $DEST_SIGN_PATH/SignConf.config
awk -i inplace -v name=`hostname` '{gsub(/emailAddress = /,"emailAddress = "name"@no-reply.com")}1' $DEST_SIGN_PATH/SignConf.config

if [ ! -f $DEST_SIGN_PATH/SignConf.config ]; then
   echo "Error: Failed to copy SignConf.config installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_SIGN_PATH
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./sign/signReadme.txt $DEST_SIGN_PATH
if [ ! -f $DEST_SIGN_PATH/signReadme.txt ]; then
   echo "Error: Failed to copy signReadme.txt installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_SIGN_PATH
   exit 1
fi
$QUIC_LN_RM_MK_DIR/chmod 644 $DEST_SIGN_PATH/signReadme.txt


if [[ $QUIC_SECURE_BOOT_CHECK = "SecureBoot enabled" ]]; then
   echo "SecureBoot enabled"
   if [ -f $DEST_SIGN_PATH/Signkey_pub.der ]; then
      if [[ $QUIC_PUBLIC_KEY_VERIFY = "$DEST_SIGN_PATH/Signkey_pub.der is already enrolled" ]]; then
         echo "==========================================================="
         echo "$DEST_SIGN_PATH/Signkey_pub.der is enrolled"
         echo "==========================================================="
      else
         echo "==========================================================="
         echo "Secure Boot is enabled. User must enroll the Public Signkey located at /opt/QUIC/sign/Signkey_pub.der"
         echo "Please follow the mandatory instructions in /opt/QUIC/sign/signReadme.txt"
         echo "The USB driver installation Failed!!!"
         echo "==========================================================="
         exit 1
      fi
   else
      echo "Signkey_pub.der doesn't exist. Creating public and private key"
      openssl req -x509 -new -nodes -utf8 -sha256 -days 36500 -batch -config $DEST_SIGN_PATH/SignConf.config -outform DER -out $DEST_SIGN_PATH/Signkey_pub.der -keyout $DEST_SIGN_PATH/Signkey.priv
      $QUIC_LN_RM_MK_DIR/chmod 755 $DEST_SIGN_PATH/Signkey_pub.der
      $QUIC_LN_RM_MK_DIR/chmod 755 $DEST_SIGN_PATH/Signkey.priv
      echo "##############################################################"
      echo "Secure Boot is enabled. User must enroll the Public Signkey located at /opt/QUIC/sign/Signkey_pub.der"
      echo "Please follow the mandatory instructions in /opt/QUIC/sign/signReadme.txt"
      echo "The USB driver installation Failed!!!"
      echo "##############################################################"
      exit 1
   fi
else
   echo "Secure boot disabled"
fi


# "********************* Compilation of modules Starts ***************************"
$QUIC_MAKE_DIR/make install
if [ ! -f ./InfParser/$QUIC_MODULE_INF_NAME ]; then
   echo "Error: Failed to generate kernel module $QUIC_MODULE_INF_NAME, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   $QUIC_MAKE_DIR/make clean
   exit 1
fi
if [ ! -f ./QdssDiag/$QUIC_MODULE_QDSS_DIAG_NAME ]; then
   echo "Error: Failed to generate kernel module $QUIC_MODULE_QDSS_DIAG_NAME, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   $QUIC_MAKE_DIR/make clean
   exit 1
fi

if [ ! -f ./rmnet/$QUIC_MODULE_RMNET_NAME ]; then
  echo "Error: Failed to generate kernel module $QUIC_MODULE_RMNET_NAME, installation abort."
  $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
  $QUIC_MAKE_DIR/make clean
  exit 1
fi

# if [ ! -f ./GobiSerial/$QUIC_MODULE_GOBISERIAL_NAME ]; then
#    echo "Error: Failed to generate kernel module $QUIC_MODULE_GOBISERIAL_NAME, installation abort."
#    $QUIC_LN_RM_MK_DIR/rm -rf $QUIC_MODULE_GOBISERIAL_NAME
#    $QUIC_MAKE_DIR/make clean
#    exit 1
# fi

# Copy .ko object to destination folders
# $QUIC_LN_RM_MK_DIR/cp ./GobiSerial/$QUIC_MODULE_GOBISERIAL_NAME $DEST_INS_SERIAL_PATH
# if [ ! -f $DEST_INS_SERIAL_PATH/$QUIC_MODULE_GOBISERIAL_NAME ]; then
#    echo "Error: Failed to copy $QUIC_MODULE_GOBISERIAL_NAME to installation path, installation abort."
#    $QUIC_LN_RM_MK_DIR/rm -r $DEST_INS_SERIAL_PATH
#    $QUIC_MAKE_DIR/make clean
#    exit 1
# fi

$QUIC_LN_RM_MK_DIR/cp ./InfParser/$QUIC_MODULE_INF_NAME $DEST_INF_PATH
if [ ! -f $DEST_INF_PATH/$QUIC_MODULE_INF_NAME ]; then
   echo "Error: Failed to copy $QUIC_MODULE_INF_NAME to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   $QUIC_MAKE_DIR/make clean
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./QdssDiag/$QUIC_MODULE_QDSS_DIAG_NAME $DEST_QDSS_DAIG_PATH
if [ ! -f $DEST_QDSS_DAIG_PATH/$QUIC_MODULE_QDSS_DIAG_NAME ]; then
   echo "Error: Failed to copy $QUIC_MODULE_QDSS_DIAG_NAME to installation path, installation abort."
   $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_QDSS_PATH
   $QUIC_MAKE_DIR/make clean
   exit 1
fi

$QUIC_LN_RM_MK_DIR/cp ./rmnet/$QUIC_MODULE_RMNET_NAME $DEST_INS_RMNET_PATH
if [ ! -f $DEST_INS_RMNET_PATH/$QUIC_MODULE_RMNET_NAME ]; then
  echo "Error: Failed to copy $QUIC_MODULE_RMNET_NAME to installation path, installation abort."
  $QUIC_LN_RM_MK_DIR/rm -rf $DEST_INS_RMNET_PATH
  exit 1
fi

$QUIC_MAKE_DIR/make clean

MODLOADED="`/sbin/lsmod | grep qtiDevInf`"
if [ "$MODLOADED" == "" ]; then
   echo "To load dependency"
   $QUIC_MODBIN_DIR/insmod $DEST_INF_PATH/$QUIC_MODULE_INF_NAME
   echo "Loading module $QUIC_MODULE_INF_NAME"
else
   echo "Module $QUIC_MODULE_INF_NAME already in place"
fi

# echo SUBSYSTEMS==\"tty\", PROGRAM=\"$DEST_INS_SERIAL_PATH/qtidev.pl $DEST_INS_SERIAL_PATH/qtiname.inf %k\", SYMLINK+=\"%c\" , MODE=\"0666\" > ./qti_usb_device.rules
echo SUBSYSTEMS==\"GobiQMI\", MODE=\"0666\" >> ./qti_usb_device.rules
echo SUBSYSTEMS==\"GobiUSB\", MODE=\"0666\" >> ./qti_usb_device.rules
echo SUBSYSTEMS==\"GobiPorts\", MODE=\"0666\" >> ./qti_usb_device.rules

$QUIC_LN_RM_MK_DIR/chmod 644 ./qti_usb_device.rules
$QUIC_LN_RM_MK_DIR/cp ./qti_usb_device.rules $QUIC_UDEV_PATH
echo "Generated QUIC rules"

# udev rules for QMI
if [ -f $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules ]; then
   RULE_EXIST="`grep -nr  'usb' $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules`"
   if [ "$RULE_EXIST" != "" ]; then
      echo "Subsystem GobiQMI rule already exist in $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules, nothing to add"
   else
      echo "Subsystem GobiQMI rule doesn't exist in $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules, so adding now"
      $QUIC_LN_RM_MK_DIR/chmod 644 $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules
      echo SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"05c6\", NAME=\"usb%n\" >> $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules
   fi
else
   echo SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"05c6\", NAME=\"usb%n\" >> ./80-gobinet-usbdevice.rules
   $QUIC_LN_RM_MK_DIR/chmod 644 ./80-gobinet-usbdevice.rules
   $QUIC_LN_RM_MK_DIR/cp ./80-gobinet-usbdevice.rules $QUIC_UDEV_PATH
   echo "Creating new udev rule for GobiQMI in $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules"
fi

# Informs udev deamon to reload the newly added device rule and re-trigger service
sudo udevadm control --reload-rules
sudo udevadm trigger

if [ ! -f $QUIC_UDEV_PATH/qti_usb_device.rules ]; then
   echo "Error: Failed to generate $QUIC_UDEV_PATH/qti_usb_device.rules"
   exit 1
fi

if [ ! -f $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules ]; then
   echo "Error: Failed to generate $QUIC_UDEV_PATH/80-gobinet-usbdevice.rules"
   exit 1
fi

rm -f ./qti_usb_device.rules
rm -f ./80-net-setup-link.rules
rm -f ./80-gobinet-usbdevice.rules
echo "Removed local rules"

MODLOADED="`/sbin/lsmod | grep usbserial`"
if [ "$MODLOADED" == "" ]; then
   echo "To load dependency"
   echo "Loading module usbserial"
   if [[ $OSName =~ "Red Hat Enterprise Linux" ]]; then
      if [ -f $QUIC_SERIAL/usbserial.ko.xz ]; then
	xz -d $QUIC_SERIAL/usbserial.ko.xz 
      	$QUIC_MODBIN_DIR/insmod $QUIC_SERIAL/usbserial.ko
      fi
      MODLOADED="`/sbin/lsmod | grep usbserial`"
      if [ "$MODLOADED" == "" ]; then
        echo "$OSName: usbserial.ko module not present at $QUIC_SERIAL"
      fi
   else
	   $QUIC_MODBIN_DIR/insmod $QUIC_SERIAL/usbserial.ko
   fi
else
   echo "Module usbserial already in place"
fi

#echo Changing Permission of blacklist file
$QUIC_LN_RM_MK_DIR/chmod 777 $MODULE_BLACKLIST/blacklist.conf
echo "Changed Permission of blacklist file"
if [ "`grep -nr 'Qualcomm clients' /etc/modprobe.d/blacklist.conf`" != "" ]; then
   sed -i '/# Blacklist these module so that Qualcomm clients use only/d' /etc/modprobe.d/blacklist.conf
   sed -i '/# GobiNet, GobiSerial, QdssDiag, qtiDevInf driver/d' /etc/modprobe.d/blacklist.conf
fi
echo "# Blacklist these module so that Qualcomm clients use only" >> /etc/modprobe.d/blacklist.conf
echo "# GobiNet, GobiSerial, QdssDiag, qtiDevInf driver" >> /etc/modprobe.d/blacklist.conf

MOD_EXIST="`grep -nr  'blacklist qcserial' /etc/modprobe.d/blacklist.conf`"
if [ "$MOD_EXIST" != "" ]; then
   sed -i '/qcserial/d' $MODULE_BLACKLIST/blacklist.conf
fi
echo "blacklist qcserial" >> /etc/modprobe.d/blacklist.conf
echo "install qcserial /bin/false" >> /etc/modprobe.d/blacklist.conf
echo "blacklisted qcserial module"

MODLOADED="`/sbin/lsmod | grep qcserial`"
if [ "$MODLOADED" != "" ]; then
   $QUIC_MODBIN_DIR/rmmod qcserial.ko
   MODLOADED="`/sbin/lsmod | grep qcserial`"
   if [ "$MODLOADED" != "" ]; then
      echo "Failed to unload qcserial.ko. Run manually sudo rmmod ModuleName"
   fi
fi
if [  -f $QUIC_SERIAL/qcserial.ko ]; then
   echo "qcserial is found. Unloaded module and moved to qcserial_dup.ko"
   mv /lib/modules/`uname -r`/kernel/drivers/usb/serial/qcserial.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/qcserial_dup.ko
fi

MOD_EXIST="`grep -nr  'blacklist qmi_wwan' /etc/modprobe.d/blacklist.conf`"
if [ "$MOD_EXIST" != "" ]; then
   sed -i '/qmi_wwan/d' $MODULE_BLACKLIST/blacklist.conf
fi
echo "blacklist qmi_wwan" >> /etc/modprobe.d/blacklist.conf
echo "install qmi_wwan /bin/false" >> /etc/modprobe.d/blacklist.conf
echo "blacklisted qmi_wwan module"

MODLOADED="`/sbin/lsmod | grep qmi_wwan`"
if [ "$MODLOADED" != "" ]; then
   $QUIC_MODBIN_DIR/rmmod qmi_wwan.ko
   $QUIC_MODBIN_DIR/rmmod cdc-wdm.ko
   MODLOADED="`/sbin/lsmod | grep qmi_wwan`"
   if [ "$MODLOADED" != "" ]; then
      echo "Failed to unload qmi_wwan.ko. Run manually sudo rmmod ModuleName"
   fi
fi
if [  -f $QUIC_QMI_WWAN/qmi_wwan.ko ]; then
   echo "qmi_wwan is found. Unloaded module and moved to qmi_wwan_dup.ko"
   mv /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-wdm.ko /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-wdm_dup.ko
   mv /lib/modules/`uname -r`/kernel/drivers/net/usb/qmi_wwan.ko /lib/modules/`uname -r`/kernel/drivers/net/usb/qmi_wwan_dup.ko
fi

MOD_EXIST="`grep -nr  'blacklist option' /etc/modprobe.d/blacklist.conf`"
if [ "$MOD_EXIST" != "" ]; then
   sed -i '/option/d' $MODULE_BLACKLIST/blacklist.conf
fi
echo "blacklist option" >> /etc/modprobe.d/blacklist.conf
echo "install option /bin/false" >> /etc/modprobe.d/blacklist.conf
echo "blacklisted option module"

MODLOADED="`/sbin/lsmod | grep option`"
if [ "$MODLOADED" != "" ]; then
   $QUIC_MODBIN_DIR/rmmod option.ko
   MODLOADED="`/sbin/lsmod | grep option`"
   if [ "$MODLOADED" != "" ]; then
      echo "Failed to unload option.ko. Run manually sudo rmmod ModuleName"
   fi
fi
if [  -f $QUIC_SERIAL/option.ko ]; then
   echo "option is found. Unloaded module and moved to option_dup.ko"
   mv /lib/modules/`uname -r`/kernel/drivers/usb/serial/option.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/option_dup.ko
fi

MOD_EXIST="`grep -nr  'blacklist usb_wwan' /etc/modprobe.d/blacklist.conf`"
if [ "$MOD_EXIST" != "" ]; then
   sed -i '/usb_wwan/d' $MODULE_BLACKLIST/blacklist.conf
fi
echo "blacklist usb_wwan" >> /etc/modprobe.d/blacklist.conf
echo "install usb_wwan /bin/false" >> /etc/modprobe.d/blacklist.conf
echo "blacklisted usb_wwan module"

MODLOADED="`/sbin/lsmod | grep usb_wwan`"
if [ "$MODLOADED" != "" ]; then
   $QUIC_MODBIN_DIR/rmmod usb_wwan.ko
   MODLOADED="`/sbin/lsmod | grep usb_wwan`"
   if [ "$MODLOADED" != "" ]; then
      echo "Failed to unload usb_wwan.ko. Run manually sudo rmmod ModuleName"
   fi
fi
if [  -f $QUIC_SERIAL/usb_wwan.ko ]; then
   echo "usb_wwan is found. Unloaded module and moved to usb_wwan_dup.ko"
   mv /lib/modules/`uname -r`/kernel/drivers/usb/serial/usb_wwan.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/usb_wwan_dup.ko
fi

MODLOADED="`/sbin/lsmod | grep GobiSerial`"
if [ "`lsmod|grep GobiSerial`" ]; then
   ( $QUIC_MODBIN_DIR/rmmod $QUIC_MODULE_GOBISERIAL_NAME && echo "$QUIC_MODULE_GOBISERIAL_NAME removed successfully." ) || { echo "$QUIC_MODULE_GOBISERIAL_NAME in use"; echo "Note: Close all applications that make use of the driver, including QUTS clients."; echo "ps -aux | grep QUTS, sudo kill -9 <PID>"; echo "Try $1ation again!"; exit 1; }
fi

# echo "Loading module $QUIC_MODULE_GOBISERIAL_NAME"
# $QUIC_MODBIN_DIR/insmod $DEST_INS_SERIAL_PATH/$QUIC_MODULE_GOBISERIAL_NAME gQTIModemInfFilePath=$QUIC_MODEM_INF_PATH debug=0

MODLOADED="`/sbin/lsmod | grep QdssDiag`"
if [ "`lsmod|grep QdssDiag`" ]; then
   ($QUIC_MODBIN_DIR/rmmod $QUIC_MODULE_QDSS_DIAG_NAME && echo "$QUIC_MODULE_QDSS_DIAG_NAME removed successfully..") ||  { echo "$QUIC_MODULE_QDSS_DIAG_NAME in use"; echo "Note: Close all applications that make use of the driver, including QUTS clients."; echo "ps -aux | grep QUTS, sudo kill -9 <PID>"; echo "Try $1ation again!"; exit 1; }
fi

echo "Loading module $QUIC_MODULE_QDSS_DIAG_NAME"
$QUIC_MODBIN_DIR/insmod $DEST_QDSS_DAIG_PATH/$QUIC_MODULE_QDSS_DIAG_NAME gQdssInfFilePath=$QUIC_QDSS_INF_PATH gDiagInfFilePath=$QUIC_DIAG_INF_PATH debug_g=1

MODLOADED="`/sbin/lsmod | grep mii`"
if [ "$MODLOADED" == "" ]; then
   echo "To load dependency"
   echo "Loading module mii"
   if [[ $OSName =~ "Red Hat Enterprise Linux" ]]; then
      if [ -f $QUIC_NET/mii.ko.xz ]; then
        xz -d $QUIC_NET/mii.ko.xz
      fi
      if [ -f $QUIC_NET/mii.ko ]; then
         $QUIC_MODBIN_DIR/insmod $QUIC_NET/mii.ko
      fi
      MODLOADED="`/sbin/lsmod | grep mii`"
      if [ "$MODLOADED" == "" ]; then
        echo "$OSName: mii.ko module not present at $QUIC_NET"
      fi
   else
      $QUIC_MODBIN_DIR/insmod $QUIC_NET/mii.ko
   fi
else
   echo "Module mii already in place"
fi

MODLOADED="`/sbin/lsmod | grep usbnet`"
if [ "$MODLOADED" == "" ]; then
   echo "To load dependency"
   echo "Loading module usbnet"
   if [[ $OSName =~ "Red Hat Enterprise Linux" ]]; then
      if [ -f $QUIC_QMI_WWAN/usbnet.ko.xz ]; then
        xz -d $QUIC_QMI_WWAN/usbnet.ko.xz
      fi
      if [ -f $QUIC_QMI_WWAN/usbnet.ko ]; then
         $QUIC_MODBIN_DIR/insmod $QUIC_QMI_WWAN/usbnet.ko
      fi
      MODLOADED="`/sbin/lsmod | grep usbnet`"
      if [ "$MODLOADED" == "" ]; then
        echo "$OSName: usbnet.ko module not present at $QUIC_QMI_WWAN"
      fi
   else
   	$QUIC_MODBIN_DIR/insmod $QUIC_QMI_WWAN/usbnet.ko
   fi
else
   echo "Module usbnet already in place"
fi
# mute rmnet
MODLOADED="`/sbin/lsmod | grep GobiNet`"
if [ "$MODLOADED" == "" ]; then
  echo "Loading module $QUIC_MODULE_RMNET_NAME"
else
  ($QUIC_MODBIN_DIR/rmmod $QUIC_MODULE_RMNET_NAME && echo "$QUIC_MODULE_RMNET_NAME removed successfully") || { echo "$QUIC_MODULE_RMNET_NAME in use"; echo "Note: Close all applications that make use of the driver, including QUTS clients."; echo "ps -aux | grep QUTS, sudo kill -9 <PID>"; echo "Try $1ation again!"; exit 1; }
fi
$QUIC_MODBIN_DIR/insmod $DEST_INS_RMNET_PATH/$QUIC_MODULE_RMNET_NAME debug_g=1 debug_aggr=0

$QUIC_MAKE_DIR/find $DEST_QUIC_PATH -type d -exec chmod 0755 {} \;

echo "Qualcomm GobiNet driver is installed at $DEST_INS_RMNET_PATH"
echo "Qualcomm INF Parser driver is installed at $DEST_INF_PATH"
echo "Qualcomm QDSS/Diag driver is installed at $DEST_QDSS_DAIG_PATH"
echo "Qualcomm Modem driver is installed at $DEST_INS_SERIAL_PATH"
echo "Qualcomm Gobi device naming rules are installed at $QUIC_UDEV_PATH"

if [ -f "$DEST_USB_PATH/ReleaseNotes*.txt" ]; then
   echo "USB Release Notes available at $DEST_USB_PATH"
fi

MODUPDATE="`grep -r qtiDevInf /etc/modules`"
if [ "$MODUPDATE" == "qtiDevInf" ]; then
  sed -i '/qtiDevInf/d' /etc/modules
fi
MODUPDATE="`grep -nr  qtiDevInf /etc/modules`"
if [ "$MODUPDATE" == "" ]; then
	echo "qtiDevInf" >> /etc/modules
fi

MODUPDATE="`grep -nr  QdssDiag /etc/modules`"
if [ "$MODUPDATE" == "" ]; then
	echo "QdssDiag" >> /etc/modules
fi

MODUPDATE="`grep -nr  GobiNet /etc/modules`"
if [ "$MODUPDATE" == "" ]; then
	echo "GobiNet" >> /etc/modules
fi

# MODUPDATE="`grep -nr  GobiSerial /etc/modules`"
# if [ "$MODUPDATE" == "" ]; then
# 	echo "GobiSerial" >> /etc/modules
# fi

if [[ $OSName != *"Red Hat Enterprise Linux"* ]]; then
   MODUPDATE="`grep -nr  'iface usb0 inet static' /etc/network/interfaces`"
   if [ "$MODUPDATE" == "" ]; then
	echo "iface usb0 inet static" >> /etc/network/interfaces
  fi
fi

exit 0
