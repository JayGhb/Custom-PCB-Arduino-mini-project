# Custom-PCB-Arduino-mini-project
Logging data, collected from custom pcb board, into SD card, using Arduino Pro mini.

The custom PCB is a break-out board for a lux, temperature and a humidity sensor, created with EAGLE 9.1.3
The ICs used are:
  * [TSL2561](https://cdn-shop.adafruit.com/datasheets/TSL2561.pdf) Lux sensor
  * [TMP102](http://www.ti.com/lit/ds/symlink/tmp102.pdf) Temperature sensor
  * [Si7021](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf) Humidity sensor

The project implements two I2C intefaces. The default Arduino Pro mini hardware one and a software I2C inteface using [SoftwareWire](https://github.com/Testato/SoftwareWire) with pins (8,9) as SDA, SCL respectively.

## JASON_TSL2561
A custom modification of the [Adafruit tsl2561 library](https://github.com/adafruit/TSL2561-Arduino-Library) is also implemented under
the JASON_TSL2561 folder, to be able to use the convenient provided functions with the SoftwareWire library.

### Other parts used
  1. Arduino Pro mini
  1. [MicroSD breakout board](https://www.sparkfun.com/products/544?_ga=2.162410361.45600573.1549733271-2101103923.1549733271)
  
Image of the board before ratsnest:


![image](https://github.com/JayGhb/Custom-PCB-Arduino-mini-project/blob/master/sample.png)
