#pragma once

/* SPI */
#define SPI_PORT spi1
#define PIN_MISO 15
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 12

/* DEBUG BUTTONS */
#define CHARGE_BTN 0
#define CHIP_BTN 1
#define KICK_BTN 2

/* HV INDICATOR LEDS */
#define HV_LED_MIN 3
#define HV_LED_LOW 4
#define HV_LED_MID 5
#define HV_LED_HIGH 6
#define HV_LED_MAX 7

/* HV CONTROL */
#define DC_DISABLE 18
#define KICK_TRIG 19
#define CHIP_TRIG 20
#define CHARGE_EN 23 // TEMP RESET TO 23
#define CHARGE_SYNC 24
#define VOLT_SENSE 29 // TEMP RESET TO 29

/* BREAKBEAM */
#define BREAK_LED 16
#define BREAK_TRIG 17
#define BREAK_SENSE 28

/* GENERAL LEDS */
#define LED_0 25
#define LED_1 26
#define LED_2 27 // TEMP RESET TO 27