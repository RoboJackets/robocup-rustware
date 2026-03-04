#pragma once

#define DEBUG 1
#define SPI_CLK_FREQUENCY 2000000
#define CHARGE_SYNC_EN 0

#define VOLT_RANGE 3.3 / 4096.0 // ADC vs Pico Vref
#define VOLT_CONVERSION 250 / 2.025 * VOLT_RANGE
#define VOLT_MAX 250
#define VOLT_MIN 10

#define CHARGE_TIME_MAX 10000 // 10s