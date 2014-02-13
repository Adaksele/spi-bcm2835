KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

ccflags-y := -I $(src)/include

obj-m := spi-bcm2835dma.o
spi-bcm2835dma-y := drivers/spi/spi-bcm2835dma_drv.o drivers/spi/spi-bcm2835dma_frag.o drivers/dma/dma-fragment.o drivers/dma/bcm2835-dma.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules modules_install

help:
	$(MAKE) -C $(KDIR) M=$(PWD) help
