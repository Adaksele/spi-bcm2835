KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

ccflags-y := -I $(src)/include

obj-m             := spi-bcm2835dma.o dma-fragment.o spi-dmafragment.o dma-bcm2835.o

spi-bcm2835dma-y  := drivers/spi/spi-bcm2835dma_drv.o
spi-bcm2835dma-y  += drivers/spi/spi-bcm2835dma_frag.o
dma-bcm2835-y     := drivers/dma/bcm2835-dma.o
dma-fragment-y    := drivers/dma/dma-fragment.o
spi-dmafragment-y := drivers/spi/spi-dmafragment.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules modules_install

help:
	$(MAKE) -C $(KDIR) M=$(PWD) help
