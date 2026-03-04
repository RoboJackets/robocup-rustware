#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "pins.hpp"
#include "config.hpp"
#include "kicker.hpp"

void init();
void startup();
void spi_irq_handler();

void read_command();
void read_voltage();
void update_spi_output();
void hv_led_out(uint8_t);
void gen_led_out(uint8_t);

void kicker_error(KickerError);

// SPI Data
volatile uint8_t rx_data = 0;
volatile bool data_ready = false;
uint8_t spi_out = 0x00;
bool new_command = false;

// Pub Vars
KickerState state = KickerState::Init;
uint64_t count = 0;
float voltage = 0;
bool charging = false;
long charge_start = to_ms_since_boot(get_absolute_time());

int main()
{
    init();
    state = KickerState::Startup;
    startup();

    // Main control loop
    while (true) {
        
        /// READ DATA
        state = KickerState::CommandIO;
        read_command();
        read_voltage();

        /// ERROR CHECKING
        // Charge Timeout
        if (charging && to_ms_since_boot(get_absolute_time()) - charge_start > CHARGE_TIME_MAX) {
            kicker_error(KickerError::ChargeTimeout);
        }


        /// DRIVE OUTPUTS
        update_spi_output();
        // Drive LEDs Basic
        uint8_t pattern = 0b00000;
        pattern |= voltage > VOLT_MIN;
        pattern |= (voltage > VOLT_MAX / 4) << 1;
        pattern |= (voltage > VOLT_MAX / 2) << 2;
        pattern |= (voltage > 3 * VOLT_MAX / 4) << 3;
        pattern |= (voltage > VOLT_MAX - VOLT_MIN) << 4;
        hv_led_out(pattern);

        count++;

        sleep_ms(1);
    }
}

void update_spi_output() {
    spi_out = ((uint8_t) voltage) >> 1;
    spi_out |= 1 << 7; // TEMP
    spi_get_hw(SPI_PORT)->dr = spi_out;
}

void read_command() {
    if (data_ready) {
        data_ready = false;
        KickerCommand command = KickerCommand(rx_data);
        #if DEBUG
            printf("Spi Out: %0x | Raw: %0x\n", spi_out, rx_data);
            command.print();
        #endif
    }
    new_command = true;
}

void read_voltage() {
    adc_select_input(VOLT_CHANNEL);
    uint16_t raw = adc_read();
    float voltage_new = VOLT_CONVERSION * raw;
    voltage = ((255 - KALPHA) * voltage + KALPHA * voltage_new) / 255;

    #if DEBUG 
        printf("Volt Raw: %d | Volt Actual: %.2f | Volt Normalized: %.2f\n", raw, voltage_new, voltage);
    #endif
}

// Check all compenents health
void startup() {
    hv_led_out(0b10001);
    sleep_ms(1000);
    hv_led_out(0b11011);
    sleep_ms(1000);
    hv_led_out(0b11111);
    sleep_ms(1000);
    hv_led_out(0b00000);
    sleep_ms(1000);
    hv_led_out(0b11111);
    sleep_ms(1000);
    hv_led_out(0b00000);
}

/* 
    Bit pattern: 000 | MAX | HIGH | MID | LOW | MIN
    1 = ON
*/
void hv_led_out(uint8_t pattern) {
    if (pattern & 0b1) {
        gpio_put(HV_LED_MIN, 0);
    } else {
        gpio_put(HV_LED_MIN, 1);
    }
    if (pattern >> 1 & 0b1) {
        gpio_put(HV_LED_LOW, 0);
    } else {
        gpio_put(HV_LED_LOW, 1);
    }
    if (pattern >> 2 & 0b1) {
        gpio_put(HV_LED_MID, 0);
    } else {
        gpio_put(HV_LED_MID, 1);
    }
    if (pattern >> 3 & 0b1) {
        gpio_put(HV_LED_HIGH, 0);
    } else {
        gpio_put(HV_LED_HIGH, 1);
    }
    if (pattern >> 4 & 0b1) {
        gpio_put(HV_LED_MAX, 0);
    } else {
        gpio_put(HV_LED_MAX, 1);
    }
}

/* 
    Bit pattern: 00000 | LED_2 | LED_1 | LED_0
    1 = ON
*/
void gen_led_out(uint8_t pattern) {
    if (pattern & 0b1) {
        gpio_put(LED_0, 0);
    } else {
        gpio_put(LED_0, 1);
    }
    if (pattern >> 1 & 0b1) {
        gpio_put(LED_1, 0);
    } else {
        gpio_put(LED_1, 1);
    }
    if (pattern >> 2 & 0b1) {
        gpio_put(LED_2, 0);
    } else {
        gpio_put(LED_2, 1);
    }
}

void init() {
    stdio_init_all();

    /// SPI initialisation
    // Mode 3, MSB First
    spi_init(SPI_PORT, SPI_CLK_FREQUENCY);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(SPI_PORT, true);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Enable interrupt
    spi_get_hw(SPI_PORT)->imsc = SPI_SSPIMSC_RXIM_BITS;

    // Set up the IRQ
    int irq = SPI_PORT == spi0 ? SPI0_IRQ : SPI1_IRQ;
    irq_set_exclusive_handler(irq, spi_irq_handler);
    irq_set_enabled(irq, true);

    // Preload output data
    spi_get_hw(SPI_PORT)->dr = spi_out;

    // Clear SPI buffers
    spi_get_hw(SPI_PORT)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
    spi_get_hw(SPI_PORT)->cr1 |= SPI_SSPCR1_SSE_BITS;


    /// Charge Sync
    GPIO_OUTPUT_INIT(CHARGE_SYNC);
    if (!CHARGE_SYNC_EN) {
        gpio_put(CHARGE_SYNC, 0);
    }

    /// Output pins
    GPIO_OUTPUT_INIT(HV_LED_MIN);
    GPIO_OUTPUT_INIT(HV_LED_LOW);
    GPIO_OUTPUT_INIT(HV_LED_MID);
    GPIO_OUTPUT_INIT(HV_LED_HIGH);
    GPIO_OUTPUT_INIT(HV_LED_MAX);
    GPIO_OUTPUT_INIT(BREAK_LED);
    GPIO_OUTPUT_INIT(BREAK_TRIG);
    GPIO_OUTPUT_INIT(DC_DISABLE);
    GPIO_OUTPUT_INIT(KICK_TRIG);
    GPIO_OUTPUT_INIT(CHIP_TRIG);
    GPIO_OUTPUT_INIT(CHARGE_EN);
    GPIO_OUTPUT_INIT(LED_0);
    GPIO_OUTPUT_INIT(LED_1);
    GPIO_OUTPUT_INIT(LED_2);

    /// Button inputs
    GPIO_INPUT_INIT(CHARGE_BTN);
    GPIO_INPUT_INIT(CHIP_BTN);
    GPIO_INPUT_INIT(KICK_BTN);

    /// ADC inputs
    adc_init();
    adc_gpio_init(BREAK_SENSE);
    adc_gpio_init(VOLT_SENSE);

}

void spi_irq_handler() {
    if (spi_is_readable(SPI_PORT)) {
        rx_data = spi_get_hw(SPI_PORT)->dr;  // read clears the interrupt
        data_ready = true;

        // Reset FIFOs
        spi_get_hw(SPI_PORT)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
        spi_get_hw(SPI_PORT)->cr1 |= SPI_SSPCR1_SSE_BITS;

        // Re-load TX FIFO with response for next transfer
        spi_get_hw(SPI_PORT)->dr = spi_out;
    }
}

// Disable charging and discharge caps
// Probably just using for errors
void shutdown() {
    gpio_put(CHARGE_EN, 0);
    sleep_ms(1);
    gpio_put(KICK_TRIG, 1);
    sleep_ms(1);
}

void kicker_error(KickerError e) {
    shutdown();
    spi_out = 0; // Send "Unhealthy" Command
    spi_get_hw(SPI_PORT)->dr = spi_out;
    gen_led_out(0b111);
    while (true) {
        printf("ERROR %s DURING %s", kicker_error_to_str(e), kicker_state_to_str(state));
        hv_led_out(e);
        sleep_ms(100);
        hv_led_out(e);
        sleep_ms(100);
    }
}