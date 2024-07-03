## Overview
This code uses the ESP32S3's SPI (GPSPI2) device to start a half-duplex octal (8 bits in parallel) transmission, sending a 25kByte segment continuously until interrupted.

The transmission can be restarted.

## Logic flow

![image](https://github.com/mrcodetastic/esp32s3_spi_dma_tx_seg_loop/assets/12006953/c6d3d43c-46eb-4b1c-8cde-4576bbba1eec)

## As seen in Pulseview
The gap between stopping a transmission, waiting for completion (via interrupt) and then restarting is only a few milliseconds.

![image](https://github.com/mrcodetastic/esp32s3_spi_dma_tx_seg_loop/assets/12006953/f69701cb-33b3-4cfb-963a-f1483dc93b91)

