# Common

## Description

This folder contains drivers and APIs for use with the Nucleo F411RE Discovery Kit while attached to the UCSC Nucleo I/O Shield.

## Contents

### `BOARD.c/h`

Contains initialization code for the Nucleo along with standard `#defines` and system libraries used. Also includes the standard fixed-width data types and error return values.

### `Adc.c/h`

Interface for using ADC channels, including the one connected to the potentiometer on the I/O Shield.

### `BNO055.c/h`

Drivers for the Adafruit BNO055 Absolute Orientation Sensor.

### `Buttons.c/h`

Provides functions used to interface with the buttons on the I/O Shield.

### `CAPTOUCH.h`

Headers providing prototypes for use with a capacitive touch sensor interface.

### `Dma.c/h`

Drivers for DMA configuration with the Nucleo STM32F411RE. You will likely not
need to call any of these functions directly as a standard user of this library.

### `Leds.c/h`

Interface for manipulating LEDs on the UCSC Nucleo I/O Shield.

### `I2c.c/h`

Library built for interfacing with I2C devices with the Nucleo.

### `Oled.c/h,  Ascii.c/h,  OledDriver.c/h`

These files provide all the code necessary for calling functions related to using the Nucleo Shield’s OLED display. You will only need to use the functions in Oled.h, the other files are called from within the Oled library.

### ``PING.h``

Header file with function prototypes for working with an ultrasonic sensor.

### `Pwm.c/h`

Drivers for use with PWM pins accessible through the UCSC Nucleo I/O Shield.

### `QEI.h`

Header file with function prototypes and macros for GPIO encoder pin definitions.

### `Timers.c/h`

Time-related functions that allow users to accurately track time with understandable units (e.g. milliseconds).

### `Uart.c/h`

Functions related to using auxiliary UART connections between Nucleos with attached I/O Shields.

### `platformio.ini`

A sample `platformio.ini` file containing an example project configuration for working with the Nucleo F411RE Discovery Kit using PlatformIO. You are free to copy this and modify it for use with your specific project needs.

### `framework_files/`

Folder containing firmware configuration definitions for use with the PlatformIO compiler. Standard users will _hopefully_ never need to modify these files.

## Metadata 

### Author

    jLab, 2026

### Date

    19 Jan 2026

### Version

    1.0.1
