#pragma once

#define DEBUG 1
#define DISABLE_ERRORS 0 // Purely for sofware debugging, do not use
#define SPI_CLK_FREQUENCY 2000000 // Hz
#define BREAK_CAL_CYCLES 3 // Number of times to measure breakbeam to set high/low
#define BTN_COOLDOWN 500 // ms

#define CHARGE_SYNC_EN 0
#define CHARGE_SYNC_FREQ 50000 // Hz (DONT GO OVER 200KHZ WILL DESTROY BOARD)

#define VOLT_RANGE 3.3 / 4096.0 // Pico Vref vs ADC
#define VOLT_CONVERSION 250 / 2.025 * VOLT_RANGE // Voltage scale with divider values
#define VOLT_MAX 180
#define VOLT_MIN 10
#define KALPHA 64 // KALHPA / 255 of last value used for averaging

#define MAX_KICK_TIME 28200 // us
#define KICK_COOLDOWN 100 // ms
#define CHARGE_COOLDOWN 100 // ms

// Error checking
#define CHARGE_TIME_MAX 20000 // ms
#define OVER_VOLTAGE 210
#define VERY_OVER_VOLTAGE 250
#define VOLT_TOLERANCE 3 // Used for comparing to old_voltage
#define VOLT_TOLERANCE_CHARGE 10
#define NO_CHARGE_COOLDOWN 1000 // ms

// Enables for specific errors
#define E_OVER_VOLTAGE 1
#define E_MAJOR_OVER_VOLTAGE 1
#define E_CONTINUOUS_CHARGING 0
#define E_CONTINUOUS_DISCHARGING 0
#define E_NO_CHARGE 0
#define E_CHARGE_TIMEOUT 1
#define E_NO_DISCHARGE 1