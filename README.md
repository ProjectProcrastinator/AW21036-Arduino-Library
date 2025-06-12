# Awinic AW21036 Arduino Library
<a href="https://registry.platformio.org/libraries/project-procrastinator/AW21036 Arduino Library"><img src="https://badges.registry.platformio.org/packages/project-procrastinator/library/AW21036 Arduino Library.svg" alt="PlatformIO Registry" /></a>

This library provides an interface for the AW21036 8-bit 36-LED driver from AWINIC. It supports almost all features of the AW21036, except for the pattern controller.

## Features
- 8-bit global current adjustment per driver
- Control up to 36 LEDs with 8-bit PWM and analog current dimming each
- Set individual or grouped RGB LEDs
- White balance and color correction
- Fast block write functions for minimal I2C traffic
- Support for broadcast and multi-driver synchronization
- Most AW21036 features implemented (except pattern controller)

## Installation
Copy the folder to your Arduino `libraries` directory.

## Usage
Include the library and needed dependancies and create an instance of the `AW21036` class. Initialize with your I2C bus and device address. Example usage is provided in the `examples` folder.

## Compatibility
The library has been tested on ESP32 using the Arduino Framework (platformio.ini is provided), but should work with all Arduino compatible devices.
Judging from the respective datasheets, this library may also be compatible with other multi-channel LED Drivers from Awinic such as AW21024. 

## Interesting findings
The AW21036 is per datasheet rated for a maximum of 400 kHz I2C frequency, but in practical testing I found that 1 MHz I2C frequency works without a problem.
