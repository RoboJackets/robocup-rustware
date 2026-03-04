#pragma once

#include <stdio.h>

#define GPIO_OUTPUT_INIT(pin) do { gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); } while(0)
#define GPIO_INPUT_INIT(pin) do { gpio_init(pin); gpio_set_dir(pin, GPIO_IN); } while(0)

#define BREAK_CHANNEL 2
#define VOLT_CHANNEL 1 // TEMP RESET TO 3

enum KickType {
    Kick,
    Chip,
};

enum KickTrigger {
    Disabled,
    Breakbeam,
    Immediate,
};

enum KickerError {
    None,

};

struct KickerCommand {
    KickType kick_type;
    KickTrigger kick_trigger;
    uint8_t kick_strength;
    bool charge_allowed;

    KickerCommand(char command) {
        kick_type = (command & (1 << 7)) ? KickType::Chip : KickType::Kick;

        char trigger_bits = (command >> 5) & 0b11;
        switch (trigger_bits) {
            case 0b01:
                kick_trigger = KickTrigger::Breakbeam;
                break;
            case 0b10:
                kick_trigger = KickTrigger::Immediate;
                break;
            default:
                kick_trigger = KickTrigger::Disabled;
        }

        charge_allowed = (command & (1 << 4)) != 0;
        kick_strength = command & 0x0F;
    }

    void print() {
        printf("Kick Type: %s | Kick Trigger: %s | Kick Strength: %d | Charge Allowed: %s\n",
                (kick_type == KickType::Chip ? "Chip" : "Kick"),
                (kick_trigger == KickTrigger::Breakbeam ? "Breakbeam" : (kick_trigger == KickTrigger::Immediate ? "Immediate" : "Disabled")),
                kick_strength,
                (charge_allowed ? "YES" : "NO"));
    }
};