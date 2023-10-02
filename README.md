# Robocup Rustware

## Description

In 2023, we decided to switch microcontrollers from the team built A-Train microcontroller to the off-the-shelf Teensy 4.1 microcontroller.  In making this switch the team also decided that a full firmware rewrite was in order.

Therefore, this repository contains the code for the firmware for the RoboJackets team.

## Common Commands

### Running main.rs

Kelvin created a runner to make running code on the teensy easier.  To utilize this runner run:

```sh
cargo run --target thumbv7em-none-eabihf
```

### examples (and tests)

To allow specific modules of our codebase to be tested in separate, we will attempt to create examples that isolate a specific device driver.  To run any of these sanity checks run:

```sh
cargo run --example <example_name> --target thumbv7em-none-eabihf
```

### Documentation

To open the documentation for internal drivers, libraries, and dependencies run:

```sh
cargo docs --open
```

## New Members Project

As you may see, the repository is currently quite bare.  This is due to the rewrite having only started this semester (and a good amount of work and poc testing being done in other repositories).  Therefore, the documentation for a new member project is relatively sparse and probably seems thrown together at the last moment (thats because it is).

Regardless, the new member project is to create an RGB Led Device Driver.

### Description

An RGB Led Device Driver is a self-contained driver utilizing the rust embedded_hal features (more information [here]()).  By using the embedded_hal features and traits, it is possible to create a driver that works regardless of the underlying architecture of the microcontroller used (i.e. you could use this driver for arduino, raspberry pi, etc.).  The main point of this driver is to provide an interface to turn the led a specific color via a singular command (plus this is largely what most of this semester's firmware development will focus on).

### Steps

1. Clone the repo

```sh
git clone https://github.com/RoboJackets/robocup-rustware.git
```

1. Branch off the repo

```sh
git checkout -b <first_name>_<last_name>-firmware_tutorial
```

1. Setup the Teensy Loader CLI

for linux

```sh
./setup/unix.sh
```

for mac

```sh
OS=MACOSX ./setup/unix.sh
```

1. Cd into the drivers folder and create a new lib crate

```sh
cd drivers
cargo new --lib <rgb_led_driver_name> 
```

1. Add the new lib crate to the [manifest file (Cargo.toml)](Cargo.toml) under workspaces
2. Write an RGB Led Driver
3. Write a test (example) in the examples folder to demo an RTIC application using the RGB Led Driver**
4. Create a PR with the RGB Led Driver
5. Demo the Driver with either Nate or Kelvin

### Writing an RGB Led Driver

The RGB Led Driver needs to have the following basic capabilities:

- Create a new driver from pins
- turn off an led
- turn a led red
- turn a led green
- turn a led blue
- turn a led yellow
- turn a led purple
- turn a led (cyan or turquoise)

feel free to experiment and add additional functionalities to the driver.  The above functionalities are only the most basic necessary features.

### Creating a Test (example)

In Rust, testing is paramount and even though this is no_std firmware, it is still useful for us to make sure that all of our drivers work in isolation.  Therefore, we'd like to have an example file (located in the examples/ directory) that contains an RTIC check that the functionalities of a driver are implemented correctly.

Ideally, every function / method in the driver should be used in the example with an optimal test making use of one driver method per software task (this isn't always possible and for the led driver it is fully allowable to have a task make the led red and then turn it off).

TLDR:

Using [main.rs](src/main.rs) as an example, write a series of software tasks utilizing the various methods of the driver you wrote in the step before.

## Device Driver Documentation

The device driver documentation can be found [here](drivers/drivers.md)

## Examples Documentation

The examples / sanity checks documentation can be found [here](examples/examples.md)

## Example Device Drivers

To give a little bit of background as to what a good device driver should kind of look like I'm going to link a few examples below:

- [OLED Display Driver](https://github.com/jamwaffles/ssd1306/tree/master) <- a bit complex but arguably Rust's most used driver
- [W25q32jv Flash Driver](https://github.com/tweedegolf/w25q32jv/tree/main) <- Ok spi flash driver
- [Radio Driver](https://github.com/astro/embedded-nrf24l01) <- Semi-working well written radio driver

## Useful Links

### Background

- [RTIC Book](https://rtic.rs/2/book/en/) <- super useful guide for RTIC
- [Embedded Rust Book](https://docs.rust-embedded.org/book/) <- Useful guide to embedded development in rust