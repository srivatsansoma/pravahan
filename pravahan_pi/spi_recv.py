import spidev
import time

spi = spidev.Spidev()

spi_bus = 0
spi_device = 0

spi.open(spi_bus,spi_device)

spi.max_speed_hz = 5e6
spi.mode = 0

while(1):
    spi.xref([0x00000000])

    time.sleep(1)
