#pragma once

#define DEBUG 1
#define SPI_CLK_FREQUENCY 2000000
#define BREAK_CAL_CYCLES 3

#define CHARGE_SYNC_EN 0
#define CHARGE_SYNC_FREQ 50000 // Hz

#define VOLT_RANGE 3.3 / 4096.0 // ADC vs Pico Vref
#define VOLT_CONVERSION 250 / 2.025 * VOLT_RANGE
#define VOLT_MAX 200
#define VOLT_MIN 10
#define KALPHA 64

#define MAX_KICK_TIME 10000 // us
#define KICK_COOLDOWN 100 // ms
#define CHARGE_COOLDOWN 100 // ms

// Error checking
#define CHARGE_TIME_MAX 40000 // ms
#define OVER_VOLTAGE 210
#define VERY_OVER_VOLTAGE 240