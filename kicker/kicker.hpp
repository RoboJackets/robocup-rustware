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
const char* kick_trigger_to_str(KickTrigger t) {
    switch(t) {
        case Breakbeam: return "Breakbeam";
        case Immediate: return "Immediate";
        default: return "Disabled";
    }
};

enum KickerError {
    None = 0b00000,
    ChargeTimeout = 0b10011,
    OverVoltage = 0b01101,
    MajorOverVoltage = 0b11111,
    ChargeKickOverlap = 0b01010,
    BreakbeamBlockage = 0b00011,
    NoCharge = 0b11100,
    NoDischarge = 0b01011,
    Unknown = 0b10101,
};

const char* kicker_error_to_str(KickerError e) {
    switch(e) {
        case None: return "None";
        case ChargeTimeout: return "ChargeTimeout";
        case OverVoltage: return "OverVoltage";
        case MajorOverVoltage: return "MAJOR OVER VOLTAGE";
        case ChargeKickOverlap: return "ChargeKickOverlap";
        case BreakbeamBlockage: return "BreakbeamBlockage";
        case NoCharge: return "NoCharge";
        case NoDischarge: return "NoDischarge";
        default: return "Unknown";
    }
};

enum KickerState {
    Charging,
    Kicking,
    Chipping,
    CommandIO,
    Init,
    Startup,
};

const char* kicker_state_to_str(KickerState s) {
    switch(s) {
        case Charging: return "Charging";
        case Kicking: return "Kicking";
        case Chipping: return "Chipping";
        case CommandIO: return "CommandIO";
        case Init: return "Init";
        case Startup: return "Startup";
        default: return "Unknown";
    }
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
                kick_trigger_to_str(kick_trigger),
                kick_strength,
                (charge_allowed ? "YES" : "NO"));
    }
};