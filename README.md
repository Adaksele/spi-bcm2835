spi-bcm2708
===========

SPI master driver with DMA support for the Raspberry Pi

See [wiki](https://github.com/notro/spi-bcm2708/wiki) for more information.

history:
--------
Original Source: https://github.com/msperl/linux/blob/rpi-3.6.y/drivers/spi/spi-bcm2708.c

Subsequently forked as an out-of-source module by notro: https://github.com/notro/spi-bcm2708

Reforked to reflect spi-bcm2835 to get it upstream

Planned enhancments:
--------------------

* DMA only driver (removal of interrupt an polling version)

* deeper DMA pipeling of SPI-transfers - currently we do one transfer and then we go thru setup again, which adds latencies.

* automatic detection of DMA addresses, which will remove the need for a bounce buffer (that could even allow direct DMA to user-space (even with spidev if it would allow to work without bounce-buffers)

* with this comes the "fact" that we will need to split a single transfer of 64k into possibly multiple DMA transfers to the correct pages.

* implement "sleeping" (spi_transfer.delay_usecs) also via DMA, so we do not have to wake up after our sleep - reducing latencies further - this way we could create a constant sampling rate for ADCs (if the driver is doing "correct" spi_transfer sequencing).





