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

pub struct RadioMod {
    settings: InternalSettings,
    return_menu_flag: bool,
}

impl RadioMod {
    pub fn new() -> RadioMod {
        
    }
}