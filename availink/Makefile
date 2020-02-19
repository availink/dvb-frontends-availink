
obj-m += avl62x1.o avl_bsp.o

avl_bsp-objs=avl_bsp_linux.o avl_lib.o
avl62x1-objs=avl62x1_top.o avl62x1_api.o avl62x1_lib.o

#EXTRA_CFLAGS += -D_AVL_LINUX_DVB_
EXTRA_CFLAGS += -I. 
EXTRA_CFLAGS += -Idrivers/media/dvb-core -Idrivers/media/usb/dvb-usb -Idrivers/media/dvb-frontends -Idrivers/media/tuners 
#EXTRA_CFLAGS += -Idrivers/amlogic/dvb-avl -Idrivers/amlogic/dvb-avl/sdk_src -Idrivers/amlogic/dvb-avl/AV201X




