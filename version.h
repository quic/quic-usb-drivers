// Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------
#ifndef VERSION_H
#define VERSION_H

#define DRIVER_VERSION "1.0.4.25"

#ifndef RHEL_RELEASE_CODE
#define RHEL_RELEASE_VERSION(a,b) (((a) << 8) + (b))
#define RHEL_RELEASE_CODE 0
#endif

#endif
