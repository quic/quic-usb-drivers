// Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

quic-usb-drivers

quic-usb-drivers is a set of 4 driver (GobiSerial, InfParser, QdssDiag and Rmnet) runs on host systems such as Ubuntu, Redhat provides connectivity to Qualcomm device through USB. These drivers are the core drivers utilized for firmware download, crash dump collection, Diag/Qdss Logging, rmnet data connectivity, etc..  These are generic USB Drivers independent of Qualcomm targets used for USB tethered connectivity of any Qualcomm target to x86 Ubuntu/Red hat systems

InfParser is a kernel module responsible for parsing INF files containing crucial information such as device type, VID/PID, and device friendly name for our Qualcomm-supported devices. This module is utilized by QdssDiag, and RMNET driver to parse device information and relay it to the kernel, ensuring the kernel is aware of supported devices beforehand.
 
QdssDiag is a USB-based kernel module leveraging the USB subsystem framework APIs. It supports various protocols including DIAG (Diagnostic), QDSS (Qualcomm Debug Subsystem), DPL (Data Protocol Logging), and modem serial communication.
 
RMNET is a USB-based network driver designed to support the WWAN connectivity and data call bringup and throughput testing. It also supports QMI control interface for assessing various services on device.

Installation/Uninstallation:
 
1.	chmod +x QcDevDriver.sh
2.	Installation - sudo ./QcDevDriver.sh install
3.	Uninstallation - sudo ./QcDevDriver.sh uninstall

