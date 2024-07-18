# Examples

## Description

Each example in this folder consists of a simple RTIC executable to sanity check the workings of various drivers and workings.  The purpose of these examples is to make firmware development based on combining things that are known to work to (ideally) create a product that we know works

## Examples

### Demo

The demo example is nothing fancy.  It sets up a basic RTIC file and spawns a software task that blinks the onboard teensy 4.1 led.

### Fpga Tuning

The FPGA Tuning example program calculates the difference between the target and actual speed of each of the wheels on the FPGA at various speeds.  It then uses the collected data points to form a line of best fit over the errors so that motion control can use these values as a starting point to obtain better measurements.

## Guidelines

- Each example should have a basic writeup both in this markdown file and as a doc comment at the top of the example file.
TODO: