# Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause

#obj-m := QdssDiag.o ../InfParser/qtiDevInf.o
obj-m := QdssDiag.o
QdssDiag-objs := qtiDiag.o qtiEvt.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all: clean
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.* modules.order *.ur-safe



