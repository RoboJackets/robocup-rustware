#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pins.hpp"
#include "config.hpp"
#include "kicker.hpp"

int main()
{
    stdio_init_all();

    // SPI initialisation
    spi_init(SPI_PORT, SPI_CLK_FREQUENCY);
    // Mode 3, MSB First
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(SPI_PORT, true);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    uint8_t rx;
    uint8_t tx = 0xFF;

    while (true) {
        spi_write_read_blocking(SPI_PORT, &tx, &rx, 1);
        KickerCommand command = KickerCommand(rx);
        printf("Raw: %0x\n", command);
        command.print();
    }
}
