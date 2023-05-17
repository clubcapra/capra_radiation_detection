# Radiation Detection with STM32F103C8T6 and M4011 Geiger-Muller Tube

This repository contains an STM32F103C8T6 code example for detecting radiation levels using an M4011 Geiger-Muller tube circuit. The code reads the radiation pulses from the Geiger-Muller tube and calculates the radiation level in counts per minute (CPM) then converts it to microsieverts per hour (µSv/h) based on a calibration index found in the Geiger-Muller tube's datasheet.

## Hardware Requirements

- Radiation detector
- Jetson Xavier

## Circuit Connection

Connect the Radiation circuit to the Jetson as follows:

- *TBD

## Software Requirements

- Platformio

## Usage

1. *TBD

## Calibration

To obtain an even more accurate radiation level measurements, it is crucial to calibrate the Geiger-Muller tube you are using with a known radiation source or reference standard. During the calibration process, measure the radiation dose in both counts per minute (CPM) and microsieverts per hour (µSv/h) and establish the appropriate conversion factor (`conversionFactor`) specific to your Geiger-Muller tube and calibration. In our case, we're using what the datasheet provided us.

## Notes

- Ensure the Geiger-Muller tube circuit is properly connected and powered.

- Remember to adjust the `conversionFactor` in the code based on your specific calibration index to obtain accurate radiation level measurements.
