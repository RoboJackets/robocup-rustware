extern crate alloc;

use core::cmp;

use robojackets_robocup_control::{Delay2, RFRadio, SharedSPI, CHANNEL};
use rtic_monotonics::{systick::Systick, Monotonic};
use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;

use robojackets_robocup_rtp::{
    control_message::{ShootMode, TriggerMode},
    ControlMessage, ControlMessageBuilder, RobotStatusMessage, Team, BASE_STATION_ADDRESSES,
    CONTROL_MESSAGE_SIZE, ROBOT_RADIO_ADDRESSES, ROBOT_STATUS_SIZE,
};

use crate::types::{InputStateUpdate, NextModule};
use crate::{
    types::{Button, Display},
    util::get_successful_ack_count,
};
use crate::{
    types::{ControllerModule, RadioState},
    util::render_status_title,
};

use crate::util::{encode_btn_state, render_text};

use ncomm_utils::packing::Packable;


struct InputState {
    btn_last: u8,
    btn_press_timeout: u32,

    first_read_flag: bool,

    joy_lx: u16,
    joy_ly: u16,
    joy_rx: u16,
    joy_ry: u16,
}

struct InternalState {
    dribbler_enabled: bool,
    kicker_state: u8,

    ball_sense_status: bool,
    kick_health: bool,
    battery_voltage: u8,
    motor_error: u8,
    fpga_status: bool,

    options_selected_entry: i8,

    input_state: InputState,

    radio_state: RadioState,

    pend_radio_config_update: bool,
}

pub struct RadioMod {
    state: InternalState,
    return_menu_flag: bool,
}

impl RadioMod {
    pub fn new() -> RadioMod {
        RadioMod { 
            state: InternalState {
                dribbler_enabled: false,
                kicker_state: 0,
            
                ball_sense_status: false,
                kick_health: false,
                battery_voltage: 0,
                motor_error: 0,
                fpga_status: false,

                options_selected_entry: 1,
            
                input_state: InputState {
                    btn_last: 0,
                    btn_press_timeout: 0,
                
                    first_read_flag: true,
                
                    joy_lx: 0,
                    joy_ly: 0,
                    joy_rx: 0,
                    joy_ry: 0,
                },
            
                radio_state: RadioState {
                    team: 0,
                    robot_id: 0,
                    conn_acks_results: [false; 100],
                    conn_acks_attempts: 0,
                },
            
                pend_radio_config_update: false,
            },
            return_menu_flag: false,
        }
    }
    //What methods do I need for radio testing?
    //ask nate what benchmark I should base this off
}

impl ControllerModule for RadioMod {
    //perform any nessesary work to update the display
    fn update_display(&self, display: Display) {
        
    }

    //triggered on button press or poll. provides the current state of the inputs.
    fn update_inputs(&mut self, input: InputStateUpdate) {

    }
    
    //perform nessesary work with the radio
    fn radio_update(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2) {

    }
    
    //update the radio settings of the module
    fn update_settings(&mut self, settings: &mut RadioState) {

    }
    
    //poll to see what module to switch to
    fn next_module(&mut self) -> NextModule {
        
    }
    
    //triggered when the module is switched to
    fn reset(&mut self) {

    }
}