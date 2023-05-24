# Radiation Detection with STM32F103C8T6 and M4011 Geiger-Muller Tube

This repository contains an STM32F103C8T6 code example for detecting radiation levels using an M4011 Geiger-Muller tube circuit. The code reads the radiation pulses from the Geiger-Muller tube and calculates the radiation level in counts per minute (CPM), then converts it to microsieverts per hour (µSv/h) based on a calibration index found in the Geiger-Muller tube's datasheet.

## Hardware Requirements

- M4011 Geiger-Muller tube
- STM32F103C8T6 microcontroller board - Called "Radiation Detector"
- Jetson Xavier (or any compatible device)
- USB-C cable

## Circuit Connection

Connect the Radiation detector and STM32F103C8T6 board to the Jetson Xavier as follows:

- Connect the M4011 Geiger-Muller tube to the Radiation Detector.
- Connect the Radiation Detector to the Jetson Xavier via USB-C cable.

*Note: The connection on the radiation detector is USB-C, but it actually uses USB 2.0 standards. We're only using it for its form factor, it also makes it easier to have the same cable everywhere in our robot when traveling.

## Software Requirements

- Platformio
- VSCode
- Optional : STLink

## Usage

1. Ensure that the Geiger-Muller tube circuit is properly connected and powered  
2. **Make sure you are within the voltage specs of the M4011 Geiger-Muller tube.**
3. Set up the development environment with Platformio or your preferred development platform. We recomment VSCode.
4. Put the bootloader onto the Radiation Detector (can be found [here](https://github.com/rogerclarkmelbourne/STM32duino-bootloader))
5. Compile and upload the code to the Radiation Detector.
6. Run the code on the board, and it will start detecting radiation levels and sending data through ROS.

## Calibration

To obtain an even more accurate measurement of radiation levels, it is crucial to calibrate the Geiger-Muller tube you are using with a known radiation source or reference standard. During the calibration process, measure the radiation dose in both counts per minute (CPM) and microsieverts per hour (µSv/h) and establish the appropriate conversion factor (`conversionRate`) specific to your Geiger-Muller tube and calibration. In this code example, we are using the conversion rate provided in the Geiger-Muller tube's datasheet.

## Notes

- Ensure that the Geiger-Muller tube circuit is properly connected and powered.
- Remember that the Tube is fragile.
- Remember to adjust the `conversionRate` in the code based on your specific calibration index to obtain accurate radiation level measurements.