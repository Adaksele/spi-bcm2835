
obj-m := spi-bcm2835dma.o
spi-bcm2835dma-y := spi-bcm2835dma_drv.o spi-bcm2835dma_frag.o DMAFragment.o bcm2835-dma.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules modules_install

help:
	$(MAKE) -C $(KDIR) M=$(PWD) help
