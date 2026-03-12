#pragma once

#define DEBUG 1
#define DISABLE_ERRORS 0 // Purely for sofware debugging, do not use
#define SPI_CLK_FREQUENCY 2000000 // Hz
#define BREAK_CAL_CYCLES 3 // Number of times to measure breakbeam to set high/low
#define BTN_DELAY 500 // ms

#define CHARGE_SYNC_EN 0
#define CHARGE_SYNC_FREQ 50000 // Hz

#define VOLT_RANGE 3.3 / 4096.0 // Pico Vref vs ADC
#define VOLT_CONVERSION 250 / 2.025 * VOLT_RANGE // Voltage scale with divider values
#define VOLT_MAX 200
#define VOLT_MIN 10
#define KALPHA 64 // KALHPA / 255 of last value used for averaging

#define MAX_KICK_TIME 10000 // us
#define KICK_COOLDOWN 100 // ms
#define CHARGE_COOLDOWN 100 // ms

// Error checking
#define CHARGE_TIME_MAX 40000 // ms
#define OVER_VOLTAGE 210
#define VERY_OVER_VOLTAGE 240
#define VOLT_TOLERANCE 5 // Used for comparing to old_voltage