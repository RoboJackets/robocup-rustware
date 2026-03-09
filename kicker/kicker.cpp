#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pins.hpp"
#include "config.hpp"
#include "kicker.hpp"

void init();
void startup();
void spi_irq_handler();
void kick(uint8_t, KickType);
KickerCommand read_command();
float read_voltage();
void update_spi_output();
void hv_led_out(uint8_t);
void gen_led_out(uint8_t);
uint16_t read_breakbeam();
void breakbeam_calibration();
void set_breakbeam(bool);
void light_show();
void manual_mode();

void kicker_error(KickerError);

// SPI Data
volatile uint8_t rx_data = 0;
volatile bool data_ready = false;
uint8_t spi_out = 0x00;

// Breakbream
uint16_t break_low = 0;
uint16_t break_high = 4095;
uint32_t break_val = 0;
bool checking_break = true;
bool break_triggered = false;

// Kicking
long last_kick = to_ms_since_boot(get_absolute_time());

// Charging
bool charging = false;
long charge_start = to_ms_since_boot(get_absolute_time());
long last_charge = to_ms_since_boot(get_absolute_time());


// Pub Vars
KickerState state = KickerState::Init;
KickerCommand command = KickerCommand(0b11100000);
uint64_t count = 0;
float voltage = 0;
float prev_voltage = 0;


int main()
{
    init();

    #if DEBUG
        if (gpio_get(CHIP_BTN) && gpio_get(KICK_BTN)) {
            printf("ENTERING MANUAL MODE");
            while (gpio_get(CHIP_BTN) || gpio_get(KICK_BTN) || gpio_get(CHARGE_BTN)) {
                gen_led_out(0b111);
                sleep_ms(100);
                gen_led_out(0);
            }
            manual_mode();
        }
    #endif

    state = KickerState::Startup;
    startup();

    // Main control loop
    while (true) {
        
        /// READ DATA
        state = KickerState::CommandIO;
        if (data_ready) {
            command = read_command();
        }
        
        prev_voltage = voltage;
        voltage = read_voltage();

        // Breakbeam trigger on falling edge
        if (command.kick_trigger == Breakbeam) {
            break_val = ((255 - KALPHA) * break_val + KALPHA * read_breakbeam()) / 255;
            #if DEBUG
                printf("Break Val: %d | Break Low: %d | Break High: %d\n", break_val, break_low, break_high);
            #endif
            if (break_val < break_low + break_high >> 3) {
                break_triggered = true;
                set_breakbeam(false);
                #if DEBUG
                    printf("BREAK_TRIGGERED");
                #endif
            }
        }

        /// ERROR CHECKING
        // Charge Timeout
        if (charging && to_ms_since_boot(get_absolute_time()) - charge_start > CHARGE_TIME_MAX) {
            kicker_error(KickerError::ChargeTimeout);
        }

        // REALLY FUCKING BAD
        if (voltage > VERY_OVER_VOLTAGE) {
            kicker_error(KickerError::MajorOverVoltage);
        }

        // Small over-voltage
        if (voltage > OVER_VOLTAGE) {
            kicker_error(KickerError::OverVoltage);
        }

        // No charging
        if (charging && !(voltage > prev_voltage)) {
            kicker_error(KickerError::NoCharge);
        }

        // Stuck charging
        if (!charging && (voltage > prev_voltage + 2)) {
            kicker_error(KickerError::ContinuousCharging);
        }



        /// DRIVE OUTPUTS
        update_spi_output();

        // Breakbeam
        set_breakbeam(command.kick_trigger == Breakbeam);

        // Charging
        // Check charge allowace, time since kick, and if charging is needed
        if (!command.charge_allowed || to_ms_since_boot(get_absolute_time()) - KICK_COOLDOWN < last_kick || charging && voltage >= VOLT_MAX) {
            if (charging) {
                last_charge = to_ms_since_boot(get_absolute_time());
            }
            charging = false;
            gpio_put(CHARGE_EN, 0);
        } else if (command.charge_allowed && to_ms_since_boot(get_absolute_time()) - KICK_COOLDOWN > last_kick && !charging && voltage < VOLT_MAX) {
            charging = true;
            gpio_put(CHARGE_EN, 1);
            #if DEBUG
                printf("Charging...\n");
            #endif
        }
        if (charging) {
            state = Charging;
        }

        // Kicking
        // Absolutely ensure charging is not active
        if (!charging && to_ms_since_boot(get_absolute_time()) - CHARGE_COOLDOWN > last_charge) {
            if (command.kick_trigger == Immediate) {
                kick(command.kick_strength, command.kick_type);
            } else if (command.kick_trigger == Breakbeam && break_triggered) {
                kick(command.kick_strength, command.kick_type);
                break_triggered = false;
            }
        }

        // Drive HV LEDs
        uint8_t pattern = 0b00000;
        pattern |= voltage > VOLT_MIN;
        pattern |= (voltage > VOLT_MAX / 4) << 1;
        pattern |= (voltage > VOLT_MAX / 2) << 2;
        pattern |= (voltage > 3 * VOLT_MAX / 4) << 3;
        pattern |= (voltage > VOLT_MAX - VOLT_MIN) << 4;
        hv_led_out(pattern);

        // Drive GP LEDs
        pattern = 0b000;
        if (command.kick_trigger != Disabled) {
            if (command.kick_type == Chip) {
                pattern |= 0b010;
            } else {
                pattern |= 0b001;
            }
        }
        if (charging) {
            pattern |= 0b100;
        }
        gen_led_out(pattern);

        count++;

        #if DEBUG
            printf("Charge Cooldown: %s | Kick Cooldown: %s", (last_charge - CHARGE_COOLDOWN < to_ms_since_boot(get_absolute_time()), "True", "False"), (last_kick - KICK_COOLDOWN < to_ms_since_boot(get_absolute_time()), "True", "False"));
        #endif

        sleep_ms(1);
    }
}

// Strength 0-15, Chip or Kick
// Has one final check to ensure charging is not active and will also detect when there is no voltage drop on activation
void kick(uint8_t strength, KickType kick_type) {
    if (charging) {
        kicker_error(KickerError::ChargeKickOverlap);
    }
    // Disable interrupts during kicking
    irq_set_enabled(SPI1_IRQ, false);

    state = KickerState::Kicking;
    uint32_t kick_time = MAX_KICK_TIME * 15 / strength;
    
    if (kick_type == Chip) {
        gpio_put(CHIP_TRIG, 1);
    } else {
        gpio_put(KICK_TRIG, 1);
    }
    sleep_us(kick_time);
    gpio_put(KICK_TRIG, 0);
    gpio_put(CHIP_TRIG, 0);

    last_kick = to_ms_since_boot(get_absolute_time());
    command.kick_trigger = Disabled;
    irq_set_enabled(SPI1_IRQ, true);

    // If no voltage drop detected error
    if (!(read_voltage() < voltage - VOLT_MIN)) {
        kicker_error(NoDischarge);
    }
}

// Sets the data on the spi to be read by the teensy
void update_spi_output() {
    spi_out = ((uint8_t) voltage) >> 1;
    spi_out |= 1 << 7; // TEMP
    spi_get_hw(SPI_PORT)->dr = spi_out;
}

// Reads the current data on the SPI, only call when data ready
KickerCommand read_command() {
    data_ready = false;
    KickerCommand new_command = KickerCommand(rx_data);
    #if DEBUG
        printf("Spi Out: %0x | Raw: %0x\n", spi_out, rx_data);
        new_command.print();
    #endif
    return new_command;
}

// Reads voltage pin and converts to voltage along with basic rolling average to smooth input
float read_voltage() {
    adc_select_input(VOLT_CHANNEL);
    uint16_t raw = adc_read();
    float voltage_new = VOLT_CONVERSION * raw;
    return ((255 - KALPHA) * voltage + KALPHA * voltage_new) / 255;

    #if DEBUG 
        printf("Volt Raw: %d | Volt Actual: %.2f | Volt Normalized: %.2f\n", raw, voltage_new, voltage);
    #endif
}

// Activates breakbeam N times and averages high and low
void breakbeam_calibration() {
    uint32_t h = 0;
    uint32_t l = 0;
    set_breakbeam(false);
    sleep_ms(100);
    for (size_t i = 0; i < BREAK_CAL_CYCLES; i++) {
        l += read_breakbeam();
        set_breakbeam(true);
        sleep_ms(500);
        h += read_breakbeam();
        sleep_ms(500);
        set_breakbeam(false);
    }
    
    break_high = h / BREAK_CAL_CYCLES;
    break_low = l / BREAK_CAL_CYCLES;

    #if DEBUG
        printf("Break Low: %d | Break High: %d\n", break_low, break_high);
    #endif

    if (break_high < break_low + 100) {
        kicker_error(BreakbeamBlockage);
    }
}

// Returns breakbeam value: 0-4095
uint16_t read_breakbeam() {
    adc_select_input(BREAK_CHANNEL);
    return adc_read();
}

// Ties together break trig and break led
void set_breakbeam(bool state) {
    gpio_put(BREAK_TRIG, state);
    gpio_put(BREAK_LED, !state);
}

// Check all compenents health
void startup() {
    breakbeam_calibration();
    light_show();

    // Run test cycle
    while (voltage < VOLT_MIN) {
        gpio_put(CHARGE_EN, 1);
        prev_voltage = voltage;
        voltage = read_voltage();
        if (!(voltage > prev_voltage)) {
            kicker_error(NoCharge);
        }
    }
    gpio_put(CHARGE_EN, 0);
    sleep_ms(CHARGE_COOLDOWN);
    prev_voltage = voltage;
    sleep_ms(100); // Artificial delay to measure hold across time
    voltage = read_voltage();

    if (voltage > prev_voltage + 2) {
        kicker_error(ContinuousCharging);
    }

    kick(15, Chip);

    // Reset values for run
    prev_voltage = 0;
    voltage = 0;
}

// Bit pattern: 000 | MAX | HIGH | MID | LOW | MIN, 1 = ON
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

// Bit pattern: 00000 | LED_2 | LED_1 | LED_0, 1 = ON
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
    irq_set_exclusive_handler(SPI1_IRQ, spi_irq_handler);
    irq_set_enabled(SPI1_IRQ, true);

    // Preload output data
    spi_get_hw(SPI_PORT)->dr = spi_out;

    // Clear SPI buffers
    spi_get_hw(SPI_PORT)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
    spi_get_hw(SPI_PORT)->cr1 |= SPI_SSPCR1_SSE_BITS;


    /// Charge Sync
    if (!CHARGE_SYNC_EN) {
        GPIO_OUTPUT_INIT(CHARGE_SYNC);
        gpio_put(CHARGE_SYNC, 0);
    } else {
        gpio_set_function(CHARGE_SYNC, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(CHARGE_SYNC);
        uint channel = pwm_gpio_to_channel(CHARGE_SYNC);

        // Calculate wrap and divider for desired frequency
        uint32_t divider16 = SYS_CLK_HZ / CHARGE_SYNC_FREQ / 4096 + (SYS_CLK_HZ % (CHARGE_SYNC_FREQ * 4096) != 0);
        if (divider16 / 16 == 0) divider16 = 16;
        uint32_t wrap = SYS_CLK_HZ * 16 / divider16 / CHARGE_SYNC_FREQ- 1;

        pwm_set_clkdiv_int_frac(slice, divider16 / 16, divider16 & 0xF);
        pwm_set_wrap(slice, wrap);
        pwm_set_chan_level(slice, channel, wrap * 0.5f);
        pwm_set_enabled(slice, true);
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

// SPI interrupt, sets data ready on new command
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
// Primarily used for error handling, consider unsafe
void hard_shutdown() {
    gpio_put(CHARGE_EN, 0);
    sleep_ms(CHARGE_COOLDOWN);
    gpio_put(KICK_TRIG, 1);
    sleep_us(MAX_KICK_TIME);
    gpio_put(KICK_TRIG, 0);
}

// Only for major over voltage event
// WILL POTENTIALLY DESTROY MOTOR BOARD
void suicide_protocal() {
    gpio_put(CHARGE_EN, 0);
    gpio_put(KICK_TRIG, 1);
    gpio_put(CHIP_TRIG, 1);
}

// Universal error handler
// Always does everything it can to discharge safely
// Assume unrecoverable
void kicker_error(KickerError e) {
    uint8_t flash_delay = 200;
    if (e == MajorOverVoltage) {
        suicide_protocal();
        flash_delay = 100;
    } else {
        hard_shutdown();
    }
    
    spi_out = 0; // Send "Unhealthy" Command
    spi_get_hw(SPI_PORT)->dr = spi_out;
    while (true) {
        if (e != MajorOverVoltage) { // Always check for extreme voltage case
            voltage = read_voltage();
            if (voltage >= VERY_OVER_VOLTAGE) {
                kicker_error(MajorOverVoltage);
            }
        } else {
            printf("I DONT WANT TO DIE\n");
        }
        printf("ERROR %s DURING %s\n", kicker_error_to_str(e), kicker_state_to_str(state));
        hv_led_out(e);
        gen_led_out(0b111);
        sleep_ms(100);
        hv_led_out(0);
        sleep_ms(100);
        gen_led_out(0);
        
    }
}

// Test all LEDs
void light_show() {
    uint8_t pattern = 0b00001111;
    for (size_t i = 0; i < 17; i++) {
        hv_led_out(pattern);
        gen_led_out(pattern >> 5);
        uint8_t t = pattern & 0b1;
        pattern = pattern >> 1;
        pattern |= t << 7;
        sleep_ms(200);
    }
    hv_led_out(0b11111);
    gen_led_out(0b111);
    sleep_ms(500);
    hv_led_out(0);
    gen_led_out(0);
    sleep_ms(500);
}

// Ignores all data except extreme voltage and charge/discharge exceptions
// BY NATURE THIS IS UNSAFE WHEN USED IMPROPERLY
void manual_mode() {
    irq_set_enabled(SPI1_IRQ, false);
    state = Manual;
    bool kick_queued = false;
    KickType kt = Kick;
    while (true) {
        // Read buttons

        if (gpio_get(CHARGE_BTN) && gpio_get(KICK_BTN) && gpio_get(CHIP_BTN)) {
            // reset vars
            charging = false;
            voltage = 0;
            hv_led_out(0);
            gen_led_out(0);
            gpio_put(CHARGE_EN, 0);
            irq_set_enabled(SPI1_IRQ, true);
            sleep_ms(10);
            return;
        }

        if (gpio_get(CHARGE_BTN)) {
            gpio_put(LED_2, 1);
            charging = true;
        } else {
            gpio_put(LED_2, 0);
            charging = false;
        }

        if (gpio_get(KICK_BTN)) {
            gpio_put(LED_0, 1);
            kick_queued = true;
            kt = Kick;
            sleep_ms(BTN_DELAY);
        } else {
            gpio_put(LED_0, 0);
        }

        if (gpio_get(CHIP_BTN)) {
            gpio_put(LED_1, 1);
            kick_queued = true;
            kt = Chip;
            sleep_ms(BTN_DELAY);
        } else {
            gpio_put(LED_1, 0);
        }

        voltage = read_voltage();

        // Drive HV LEDs
        uint8_t pattern = 0b00000;
        pattern |= voltage > VOLT_MIN;
        pattern |= (voltage > VOLT_MAX / 4) << 1;
        pattern |= (voltage > VOLT_MAX / 2) << 2;
        pattern |= (voltage > 3 * VOLT_MAX / 4) << 3;
        pattern |= (voltage > VOLT_MAX - VOLT_MIN) << 4;
        hv_led_out(pattern);

        if (voltage == VERY_OVER_VOLTAGE) {
            kicker_error(MajorOverVoltage);
        }

        if (charging) {
            gpio_put(CHARGE_EN, 1);
        } else {
            gpio_put(CHARGE_EN, 0);
            if (kick_queued) {
                sleep_ms(CHARGE_COOLDOWN);
                gpio_put((kt == Kick ? KICK_TRIG : CHIP_TRIG), 1);
                sleep_us(MAX_KICK_TIME);
                gpio_put((kt == Kick ? KICK_TRIG : CHIP_TRIG), 0);
                sleep_ms(KICK_COOLDOWN);
                kick_queued = false;
            }
        }
        sleep_ms(1);
    }
}