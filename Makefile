KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

ccflags-y := -I $(src)/include

obj-m                += spi-bcm2708.o
obj-m                += spi-bcm2835.o

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
