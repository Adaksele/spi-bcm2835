KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

ccflags-y := -I $(src)/include

obj-m                := spi-bcm2835dma.o
spi-bcm2835dma-y     := drivers/spi/spi-bcm2835dma_drv.o
spi-bcm2835dma-y     += drivers/spi/spi-bcm2835dma_frag.o

obj-m                += dma-fragment.o dma-fragment-debug.o
dma-fragment-y       := drivers/dma/dma-fragment.o
dma-fragment-debug-y := drivers/dma/dma-fragment-debug.o

obj-m                += spi-dmafragment.o
spi-dmafragment-y    := drivers/spi/spi-dmafragment.o

obj-m                += dma-bcm2835.o dma-bcm2835-debug.o
dma-bcm2835-y        := drivers/dma/bcm2835-dma.o
dma-bcm2835-debug-y  := drivers/dma/bcm2835-dma-debug.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules modules_install

help:
	$(MAKE) -C $(KDIR) M=$(PWD) help

checkpatch:
	find include drivers -type f -name "*.[ch]" \
	| xargs -- $(KDIR)/scripts/checkpatch.pl \
	--emacs --no-tree --file --show-types --terse \
	--ignore MULTISTATEMENT_MACRO_USE_DO_WHILE,SPLIT_STRING,BRACES
