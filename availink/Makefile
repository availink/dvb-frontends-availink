
obj-m += avl62x1.o avl_bsp.o avl68x2.o

avl_bsp-objs=avl_bsp_linux.o avl_lib.o
avl62x1-objs=avl62x1_top.o avl62x1_api.o avl62x1_lib.o
avl68x2-objs=avl68x2_top.o avl68x2_dvbsx.o avl68x2_dvbtx.o avl68x2_internal.o avl68x2_common.o avl68x2_isdbt.o avl68x2_dvbc.o


#EXTRA_CFLAGS += -D_AVL_LINUX_DVB_
EXTRA_CFLAGS += -I. 
EXTRA_CFLAGS += -Idrivers/media/dvb-core -Idrivers/media/usb/dvb-usb -Idrivers/media/dvb-frontends -Idrivers/media/tuners 
#EXTRA_CFLAGS += -Idrivers/amlogic/dvb-avl -Idrivers/amlogic/dvb-avl/sdk_src -Idrivers/amlogic/dvb-avl/AV201X




