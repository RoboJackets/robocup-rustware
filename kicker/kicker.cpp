#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pins.hpp"
#include "config.hpp"
#include "kicker.hpp"

void init();
void spi_irq_handler();

// SPI Data
volatile uint8_t rx_data = 0;
volatile bool data_ready = false;
uint8_t output = 0;


int main()
{
    init();

    while (true) {
        // Read new command
        if (data_ready) {
            data_ready = false;
            KickerCommand command = KickerCommand(rx_data);
            if (DEBUG) {
                printf("Count: %d | Raw: %0x\n", output, rx_data);
                command.print();
            }
        }

        
    }
}

void init() {
    stdio_init_all();

    // SPI initialisation
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
    irq_set_exclusive_handler(SPI1_IRQ, spi_irq_handler);
    irq_set_enabled(SPI1_IRQ, true);

    // Preload output data
    spi_get_hw(SPI_PORT)->dr = output;

    // Clear SPI buffers
    spi_get_hw(SPI_PORT)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
    spi_get_hw(SPI_PORT)->cr1 |= SPI_SSPCR1_SSE_BITS;
    
}

void spi_irq_handler() {
    if (spi_is_readable(SPI_PORT)) {
        rx_data = spi_get_hw(SPI_PORT)->dr;  // read clears the interrupt
        data_ready = true;

        // Reset FIFOs
        spi_get_hw(SPI_PORT)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
        spi_get_hw(SPI_PORT)->cr1 |= SPI_SSPCR1_SSE_BITS;

        // Re-load TX FIFO with response for next transfer
        spi_get_hw(SPI_PORT)->dr = output;
    }
}