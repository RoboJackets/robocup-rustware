#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pins.hpp"
#include "config.hpp"
#include "kicker.hpp"
#include <cmath>

void init();
void startup();
void light_show();
void spi_irq_handler();
KickerCommand read_command();
uint16_t read_breakbeam();
uint16_t read_voltage_raw();
float read_voltage();
void update_spi_output();
void hv_led_out(uint8_t);
void gen_led_out(uint8_t);
void kick(uint8_t, KickType);
void hard_shutdown();
void suicide_protocal();
void kicker_error(KickerError);
void manual_mode();

// Breakbream
volatile bool checking_break = true;
volatile bool break_triggered = false;
volatile bool break_raw = false;

// SPI Data
volatile uint8_t rx_data = 0;
volatile bool data_ready = false;
uint8_t spi_out = 0x00;

// Buttons
long kick_btn_cooldown = 0;
long chip_btn_cooldown = 0;
long charge_btn_cooldown = 0;

// Kicking
uint64_t last_kick = to_ms_since_boot(get_absolute_time());

// Charging
bool charging = false;
uint64_t charge_start = to_ms_since_boot(get_absolute_time());
uint64_t last_charge = to_ms_since_boot(get_absolute_time());

// Shared Vars
KickerState state = KickerState::Init;
KickerCommand command = KickerCommand(0b00000000); // Start with no charge and disabled
uint64_t count = 0;
float voltage = 0;
float prev_voltage = 0;
float old_voltage = 0;
mutex_t adc_mutex;
uint64_t watchdog_time;

// Breakbeam reading
void core1_entry() {
    while(true) {
        // OFF
        int16_t break_off = read_breakbeam();
        gpio_put(BREAK_TRIG, 1);
        sleep_ms(5);

        // ON
        int16_t break_on = read_breakbeam();
        gpio_put(BREAK_TRIG, 0);
        sleep_ms(20);

        int16_t diff = abs(break_off - break_on);

        if (diff < BREAK_THRESHOLD) {
            gpio_put(BREAK_LED, 0);
            if (checking_break) {
                break_triggered = true;
            }
            break_raw = true;
        } else {
            gpio_put(BREAK_LED, 1);
            break_raw = false;
        }

        #if EXTRA_INFO && DEBUG
            printf("Break Off: %d, Break On: %d, Difference: %d\n", break_off, break_on, diff);
        #endif
    }
}

// Main control
int main() {
    // Disable interrupts during setup
    irq_set_enabled(SPI1_IRQ, false);
    init();

    #if DEBUG
        if (gpio_get(CHIP_BTN) && gpio_get(KICK_BTN)) {
            printf("ENTERING MANUAL MODE\n");
            while (gpio_get(CHIP_BTN) || gpio_get(KICK_BTN) || gpio_get(CHARGE_BTN)) {
                gen_led_out(0b111);
                sleep_ms(100);
                gen_led_out(0);
                sleep_ms(100);
            }
            manual_mode();
        }
    #endif

    state = KickerState::Startup;
    // Debug Suite
    startup();
    irq_set_enabled(SPI1_IRQ, true);

    printf("========================MAIN PROCESS BEGIN========================\n");
    // Command default
    command.charge_allowed = false;
    command.kick_strength = 0;
    command.kick_trigger = Disabled;
    command.kick_type = Kick;

    watchdog_time = to_ms_since_boot(get_absolute_time());

    // Main control loop
    while (true) {
        /// READ DATA
        state = KickerState::CommandIO;

        if (data_ready) {
            command = read_command();
            watchdog_time = to_ms_since_boot(get_absolute_time());
        }

        if (watchdog_time + WATCHDOG_TIMEOUT < to_ms_since_boot(get_absolute_time())) {
            command.kick_trigger = Disabled;
        }

        // Read buttons
        if (kick_btn_cooldown + BTN_COOLDOWN < to_ms_since_boot(get_absolute_time()) && gpio_get(KICK_BTN)) {
            command.kick_type = Kick;
            command.kick_trigger = Immediate;
            command.kick_strength = 15;
            kick_btn_cooldown = to_ms_since_boot(get_absolute_time());
        }
        if (chip_btn_cooldown + BTN_COOLDOWN < to_ms_since_boot(get_absolute_time()) && gpio_get(CHIP_BTN)) {
            command.kick_type = Chip;
            command.kick_trigger = Immediate;
            command.kick_strength = 15;
            chip_btn_cooldown = to_ms_since_boot(get_absolute_time());
        }
        if (charge_btn_cooldown + BTN_COOLDOWN < to_ms_since_boot(get_absolute_time()) && gpio_get(CHARGE_BTN)) {
            command.charge_allowed = !command.charge_allowed;
            charge_btn_cooldown = to_ms_since_boot(get_absolute_time());
        }
        
        // Read voltage staggar previous values
        voltage = read_voltage();
        if (count % 111 == 0) {
                prev_voltage = voltage;
        }
        if (count % 200 == 0) {
                old_voltage = prev_voltage;
        }

        /// ERROR CHECKING
        // Charge Timeout
        if (E_CHARGE_TIMEOUT && charging && to_ms_since_boot(get_absolute_time()) - charge_start > CHARGE_TIME_MAX) {
            kicker_error(KickerError::ChargeTimeout);
        }

        // REALLY FUCKING BAD
        if (E_MAJOR_OVER_VOLTAGE && voltage > VERY_OVER_VOLTAGE) {
            kicker_error(KickerError::MajorOverVoltage);
        }

        // Small over-voltage
        if (E_OVER_VOLTAGE && voltage > OVER_VOLTAGE) {
            kicker_error(KickerError::OverVoltage);
        }

        // No charging
        if (E_NO_CHARGE && charging && !(voltage > old_voltage - VOLT_TOLERANCE) && to_ms_since_boot(get_absolute_time()) - charge_start > NO_CHARGE_COOLDOWN) {
            kicker_error(KickerError::NoCharge);
        }

        // Stuck charging
        if (E_CONTINUOUS_CHARGING && !charging && (voltage > old_voltage + VOLT_TOLERANCE)) {
            kicker_error(KickerError::ContinuousCharging);
        }

        // Discharging 
        if (E_CONTINUOUS_DISCHARGING && !charging && (voltage < old_voltage - VOLT_TOLERANCE) && !(last_kick + KICK_COOLDOWN > to_ms_since_boot(get_absolute_time()))) {
            kicker_error(KickerError::ContinuousDischarge);
        }

        /// DRIVE OUTPUTS
        update_spi_output();

        // Allow setting the break trigger
        checking_break = command.kick_trigger == Breakbeam && to_ms_since_boot(get_absolute_time()) - 1000 > last_kick;

        // Charging
        // Check charge allowace, time since kick, and if charging is needed
        if (!command.charge_allowed || to_ms_since_boot(get_absolute_time()) - KICK_COOLDOWN < last_kick || charging && voltage >= VOLT_MAX) {
            if (charging) {
                last_charge = to_ms_since_boot(get_absolute_time());
            }
            charging = false;
            gpio_put(CHARGE_EN, 0);
        } else if (command.charge_allowed && to_ms_since_boot(get_absolute_time()) - KICK_COOLDOWN > last_kick && !charging && voltage < VOLT_MAX - VOLT_TOLERANCE_CHARGE && to_ms_since_boot(get_absolute_time()) - CHARGE_COOLDOWN > last_charge) {
            charging = true;
            charge_start = to_ms_since_boot(get_absolute_time());
            gpio_put(CHARGE_EN, 1);
            printf("BEGINNING CHARGE\n");
        }
        if (charging) {
            state = Charging;
            gpio_put(DISCHARGE_DISABLE, 1);
        }

        // Kicking
        // Absolutely ensure charging is not active
        if (!charging && to_ms_since_boot(get_absolute_time()) - CHARGE_COOLDOWN > last_charge && to_ms_since_boot(get_absolute_time()) - KICK_COOLDOWN > last_kick) {
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
                pattern |= 0b100;
            }
        }
        if (charging) {
            pattern |= 0b001;
        }
        gen_led_out(pattern);


        #if DEBUG
            if (count % 1 == 0) {
                uint64_t sys_time = to_ms_since_boot(get_absolute_time());
                printf("======================Cycle:%llu======================\n", count);
                printf("Sys Time: %llu\n", sys_time);
                printf("Command: ");
                command.print();
                printf("Voltage: %.2f | Old Voltage: %.2f\n", voltage, old_voltage);
                printf("Charge Cooldown: %d\n", (CHARGE_COOLDOWN + last_charge > sys_time ? CHARGE_COOLDOWN - (sys_time - last_charge) : 0));
                printf("Kick Cooldown: %d\n", (KICK_COOLDOWN + last_kick > sys_time ? KICK_COOLDOWN - (sys_time - last_kick) : 0));
                printf("Charging: %s\n", (charging ? "TRUE" : "FALSE"));
                printf("Break Checking: %s | Break Triggered: %s\n", (checking_break ? "TRUE" : "FALSE"), (break_triggered ? "TRUE" : "FALSE"));
                printf("=======================================================\n");
            }
        #endif

        count++;
        sleep_ms(1);
    }
}

// Initializes all I/O and standardizes output
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
    GPIO_OUTPUT_INIT(DISCHARGE_DISABLE);
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
    mutex_init(&adc_mutex);

    // Start LEDs off
    hv_led_out(0);
    gen_led_out(0);
    gpio_put(BREAK_LED, 1);
    // Start discharge disabled
    gpio_put(DISCHARGE_DISABLE, 1);
    
    // Start breakbeam core
    multicore_launch_core1(core1_entry);
}

// Check all compenents health
void startup() {
    // Test LEDs + give delay for serial setup
    light_show();

    // Init averaged values
    #if DEBUG
    printf("INIT AVERAGED VOLTAGE\n");
    #endif
    sleep_ms(100);
    for (size_t i = 0; i < 20; i++) {
        voltage = read_voltage();
        #if DEBUG && EXTRA_INFO
            printf("Voltage: %.2f\n", voltage);
        #endif
        sleep_ms(1);
    }

    // Test Breakbeam
    checking_break = true;
    sleep_ms(100);
    if (E_BREAK_BLOCKAGE && break_triggered) {
        kicker_error(BreakbeamBlockage);
    }
    checking_break = false;
    break_triggered = false;

    // Run test cycle
    uint64_t debug_time = to_ms_since_boot(get_absolute_time());
    while (voltage < VOLT_MIN * 3) {
        gpio_put(CHARGE_EN, 1);
        voltage = read_voltage();
        if (to_ms_since_boot(get_absolute_time()) - debug_time > 5000) {
            if (E_NO_CHARGE) {
                kicker_error(NoCharge);
            } else {
                break;
            }
        }
    }
    gpio_put(CHARGE_EN, 0);
    sleep_ms(CHARGE_COOLDOWN);
    old_voltage = voltage;
    sleep_ms(100); // Artificial delay to measure hold across time
    voltage = read_voltage();

    if (E_CONTINUOUS_CHARGING && voltage > old_voltage + VOLT_TOLERANCE) {
        kicker_error(ContinuousCharging);
    }

    kick(15, Kick);

    // reset vals to return
    voltage = read_voltage();
    old_voltage = voltage;
    prev_voltage = voltage;
}

// Test all LEDs
void light_show() {
    hv_led_out(0b11111);
    gen_led_out(0b111);
    sleep_ms(150);
    hv_led_out(0b01010);
    gen_led_out(0b101);
    sleep_ms(150);
    hv_led_out(0b10101);
    gen_led_out(0b010);
    sleep_ms(150);
    hv_led_out(0b11111);
    gen_led_out(0b111);
    sleep_ms(150);
    hv_led_out(0);
    gen_led_out(0);
    sleep_ms(150);
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

// Reads the current data on the SPI, only call when data ready
KickerCommand read_command() {
    data_ready = false;
    KickerCommand new_command = KickerCommand(rx_data);
    #if DEBUG
        printf("NEW COMMAND:\nSpi Out: %0x | Raw: %0x\n", spi_out, rx_data);
    #endif
    new_command.print();
    return new_command;
}

// Returns breakbeam value: 0-4095
uint16_t read_breakbeam() {
    mutex_enter_blocking(&adc_mutex);
    adc_select_input(BREAK_CHANNEL);
    uint16_t break_val = adc_read();
    mutex_exit(&adc_mutex);
    return break_val;
}

// Returns voltage value: 0-4095
uint16_t read_voltage_raw() {
    mutex_enter_blocking(&adc_mutex);
    adc_select_input(VOLT_CHANNEL);
    uint16_t voltage_val = adc_read();
    mutex_exit(&adc_mutex);
    return voltage_val;
}

// Reads voltage pin and converts to voltage along with basic rolling average to smooth input
float read_voltage() {
    uint16_t raw = read_voltage_raw();
    float voltage_new = VOLT_CONVERSION * raw;
    float voltage_norm = ((255 - KALPHA) * voltage + KALPHA * voltage_new) / 255;
    #if DEBUG && EXTRA_INFO
        printf("Volt Raw: %d | Volt Actual: %.2f | Volt Normalized: %.2f\n", raw, voltage_new, voltage_norm);
    #endif

    return voltage_norm;
}

// Sets the data on the spi to be read by the teensy
void update_spi_output() {
    spi_out = ((uint8_t) voltage) >> 1;
    spi_out |= break_raw << 7;
    spi_get_hw(SPI_PORT)->dr = spi_out;
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
        gpio_put(LED_0, 1);
    } else {
        gpio_put(LED_0, 0);
    }
    if (pattern >> 1 & 0b1) {
        gpio_put(LED_1, 1);
    } else {
        gpio_put(LED_1, 0);
    }
    if (pattern >> 2 & 0b1) {
        gpio_put(LED_2, 1);
    } else {
        gpio_put(LED_2, 0);
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
    // Allow discharge
    gpio_put(DISCHARGE_DISABLE, 0);

    state = KickerState::Kicking;
    uint32_t kick_time = (MAX_KICK_TIME * strength * strength) / 225;
    printf("KICKING!!!!!!!!!!!!!!!!!!!!!!!!!\n");
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
    gpio_put(DISCHARGE_DISABLE, 1);
    irq_set_enabled(SPI1_IRQ, true);

    // If no voltage drop detected error
    adc_select_input(VOLT_CHANNEL);
    uint16_t raw = adc_read();
    float temp_volt = VOLT_CONVERSION * raw;
    if (E_NO_DISCHARGE && !(temp_volt < voltage - VOLT_MIN + VOLT_TOLERANCE) && !(temp_volt - voltage < 5 && temp_volt - voltage > -5)) {
        kicker_error(NoDischarge);
    }
    
    // Reset values to prevent wrong errors
    voltage = read_voltage();
    old_voltage = voltage;
}

// Disable charging and discharge caps
// Primarily used for error handling, consider unsafe
void hard_shutdown() {
    gpio_put(BREAK_TRIG, 0);
    gpio_put(BREAK_LED, 1);
    gpio_put(CHARGE_EN, 0);
    gpio_put(DISCHARGE_DISABLE, 0);
    sleep_ms(CHARGE_COOLDOWN);
    gpio_put(KICK_TRIG, 1);
    sleep_us(MAX_KICK_TIME);
    gpio_put(KICK_TRIG, 0);
    sleep_ms(KICK_COOLDOWN);
    gpio_put(DISCHARGE_DISABLE, 1);
}

// Only for major over voltage event
// WILL POTENTIALLY DESTROY MOTOR BOARD
void suicide_protocal() {
    gpio_put(BREAK_TRIG, 0);
    gpio_put(BREAK_LED, 1);
    gpio_put(CHARGE_EN, 0);
    gpio_put(DISCHARGE_DISABLE, 0);
    gpio_put(KICK_TRIG, 1);
    gpio_put(CHIP_TRIG, 1);
}

// Universal error handler
// Always does everything it can to discharge safely
// Assume unrecoverable
void kicker_error(KickerError e) {
    if (DISABLE_ERRORS) { return; }

    uint8_t flash_delay = 200;
    if (e == MajorOverVoltage) {
        suicide_protocal();
        flash_delay = 75;
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
        sleep_ms(flash_delay);
        hv_led_out(0);
        gen_led_out(0);
        sleep_ms(flash_delay);
    }
}

// Ignores all data except extreme voltage and charge/discharge exceptions
// BY NATURE THIS IS UNSAFE WHEN USED IMPROPERLY
void manual_mode() {
    irq_set_enabled(SPI1_IRQ, false);
    gpio_put(DISCHARGE_DISABLE, 0); // Always allow discharge just for fun
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

        if (charge_btn_cooldown + BTN_COOLDOWN < to_ms_since_boot(get_absolute_time()) && gpio_get(CHARGE_BTN)) {
            gpio_put(LED_2, 0);
            charging = true;
            charge_btn_cooldown = to_ms_since_boot(get_absolute_time());
        } else {
            gpio_put(LED_2, 1);
            charging = false;
        }

        if (kick_btn_cooldown + BTN_COOLDOWN < to_ms_since_boot(get_absolute_time()) && gpio_get(KICK_BTN)) {
            gpio_put(LED_0, 0);
            kick_queued = true;
            kt = Kick;
            kick_btn_cooldown = to_ms_since_boot(get_absolute_time());
        } else {
            gpio_put(LED_0, 1);
        }

        if (chip_btn_cooldown + BTN_COOLDOWN < to_ms_since_boot(get_absolute_time()) && gpio_get(CHIP_BTN)) {
            gpio_put(LED_1, 0);
            kick_queued = true;
            kt = Chip;
            chip_btn_cooldown = to_ms_since_boot(get_absolute_time());
        } else {
            gpio_put(LED_1, 1);
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