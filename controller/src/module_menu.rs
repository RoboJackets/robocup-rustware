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

use crate::types::{InputStateUpdate, NextModule, MODULE_COUNT, MODULE_ENTRIES, TEAM_NAME_MAP};
use crate::{
    types::{Button, Display},
    util::get_successful_ack_count,
};
use crate::{
    types::{ControllerModule, RadioState},
    util::render_status_title,
};

use crate::util::{encode_btn_state, render_text};

struct InputState {
    btn_last: u8,
    btn_press_timeout: u32,
    first_read_flag: bool,
}

enum Screen {
    Main = 0,
    Options = 1,
}

pub struct MenuMod {
    current_screen: Screen,
    options_selected_entry: i8,
    radio_state: RadioState,
    input_state: InputState,
    next_module: NextModule,
}

impl MenuMod {
    pub fn new() -> Self {
        Self {
            current_screen: Screen::Main,
            options_selected_entry: 0,
            radio_state: RadioState {
                team: 0,
                robot_id: 0,
                conn_acks_results: [false; 100],
                conn_acks_attempts: 0,
            },
            input_state: InputState {
                btn_last: 0,
                btn_press_timeout: 0,
                first_read_flag: true,
            },
            next_module: NextModule::None,
        }
    }

    fn btn_rising(&mut self, old_state: u8, new_state: u8, button: Button) -> bool {
        let mask = 1 << button as u8;
        //This is ~500 ms
        if (old_state & mask) == 0 && (new_state & mask) != 0 {
            self.input_state.btn_press_timeout = Systick::now().ticks() + 500;
            return true;
        } else {
            return false;
        }
    }

    fn btn_held(&mut self, old_state: u8, new_state: u8, button: Button) -> bool {
        let mask = 1 << button as u8;
        if Systick::now().ticks() < self.input_state.btn_press_timeout {
            return false;
        }
        self.input_state.btn_press_timeout = 0;
        (old_state & mask) != 0 && (new_state & mask) != 0
    }

    fn handle_entry_modify(&mut self, increment: bool) {
        match self.current_screen {
            Screen::Main => {
                match self.options_selected_entry {
                    0 => {
                        self.current_screen = Screen::Options;
                        self.options_selected_entry = 0;
                    }
                    _ => {
                        //this is a signal to change to the next module
                        self.next_module = MODULE_ENTRIES[self.options_selected_entry as usize].id;
                    }
                }
            }
            Screen::Options => match self.options_selected_entry {
                0 => {
                    self.current_screen = Screen::Main;
                    self.options_selected_entry = 0;
                }
                1 => {
                    self.radio_state.team = if self.radio_state.team == 1 { 0 } else { 1 };
                }
                2 => {
                    self.radio_state.robot_id = if increment {
                        cmp::min(self.radio_state.robot_id + 1, 11)
                    } else {
                        cmp::max(self.radio_state.robot_id - 1, 0)
                    };
                }
                _ => {}
            },
        }
    }

    fn update_buttons(&mut self, buttons: u8) {
        //get the buttons difference
        let old_state = self.input_state.btn_last;
        self.input_state.btn_last = buttons;

        match self.current_screen {
            Screen::Main => {
                if self.btn_rising(old_state, buttons, Button::Up) {
                    self.options_selected_entry = cmp::max(self.options_selected_entry - 1, 0);
                }
                if self.btn_rising(old_state, buttons, Button::Down) {
                    self.options_selected_entry =
                        cmp::min(self.options_selected_entry + 1, MODULE_COUNT as i8 - 1);
                }

                if self.btn_rising(old_state, buttons, Button::Right) {
                    self.handle_entry_modify(true);
                }

                if self.btn_rising(old_state, buttons, Button::Left) {
                    self.handle_entry_modify(false);
                }
            }
            Screen::Options => {
                if self.btn_rising(old_state, buttons, Button::Up) {
                    self.options_selected_entry = cmp::max(self.options_selected_entry - 1, 0);
                }
                if self.btn_rising(old_state, buttons, Button::Down) {
                    self.options_selected_entry = cmp::min(self.options_selected_entry + 1, 2);
                }

                if self.btn_rising(old_state, buttons, Button::Right)
                    || self.btn_held(old_state, buttons, Button::Right)
                {
                    self.handle_entry_modify(true);
                }

                if self.btn_rising(old_state, buttons, Button::Left)
                    || self.btn_held(old_state, buttons, Button::Left)
                {
                    self.handle_entry_modify(false);
                }
            }
        }
    }

    fn render_main_screen(&self, display: Display) {
        let entry_count: i32 = MODULE_COUNT as i32;
        let page_size: i32 = cmp::min(5, entry_count);
        let start: i32 = cmp::min(entry_count - page_size, self.options_selected_entry as i32);

        for i in 0..page_size {
            let entry = start + i;
            if entry < 0 {
                continue;
            }

            let highlight = entry == self.options_selected_entry as i32;
            let text = match entry {
                //this overrides the "Menu" entry in MODULE_ENTRIES, since there is no need
                //to go from the main menu to the main menu
                0 => "Settings",
                _ => MODULE_ENTRIES[entry as usize].name,
            };

            render_text(display, text, 0, i as u8 * 10 + 14, highlight);

            let arrow_text = ">";
            render_text(display, arrow_text, 104, i as u8 * 10 + 14, highlight);
        }
    }

    fn render_options_screen(&self, display: Display) {
        let entry_names = ["Back", "Team", "Robot ID"];
        let entry_values = [
            ">",
            TEAM_NAME_MAP[self.radio_state.team as usize],
            &alloc::fmt::format(format_args!("{}", self.radio_state.robot_id)),
        ];

        for i in 0..3 {
            let entry = i;
            if entry < 0 {
                continue;
            }

            let highlight = entry == self.options_selected_entry as i32;
            let entry_name = entry_names[entry as usize];
            let entry_value = entry_values[entry as usize];
            render_text(display, entry_name, 0, i as u8 * 10 + 14, highlight);
            render_text(display, entry_value, 104, i as u8 * 10 + 14, highlight);
        }
    }
}

impl ControllerModule for MenuMod {
    fn update_display(&self, display: Display) {
        let screen_name = match self.current_screen {
            Screen::Main => "Main Menu",
            Screen::Options => "Radio Opt.",
        };
        render_status_title(display, screen_name);
        match self.current_screen {
            Screen::Main => self.render_main_screen(display),
            Screen::Options => self.render_options_screen(display),
        }
    }

    fn update_inputs(&mut self, inputs: InputStateUpdate) {
        //only update buttons if all are present
        if inputs.btn_left.is_some()
            && inputs.btn_right.is_some()
            && inputs.btn_up.is_some()
            && inputs.btn_down.is_some()
        {
            let new_state = encode_btn_state(
                inputs.btn_left.unwrap(),
                inputs.btn_right.unwrap(),
                inputs.btn_up.unwrap(),
                inputs.btn_down.unwrap(),
            );

            //we don't want edge triggers on the first read
            if self.input_state.first_read_flag {
                self.input_state.first_read_flag = false;
                self.input_state.btn_last = new_state;
            }

            self.update_buttons(new_state);
        }
    }

    fn radio_update(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2) {
        //do nothing
    }

    fn update_settings(&mut self, settings: &mut RadioState) {
        //this module writes to the settings
        settings.robot_id = self.radio_state.robot_id;
        settings.team = self.radio_state.team;

        settings.conn_acks_attempts = self.radio_state.conn_acks_attempts;
        settings.conn_acks_results = self.radio_state.conn_acks_results;
    }

    fn next_module(&mut self) -> NextModule {
        self.next_module
    }

    fn reset(&mut self) {
        self.next_module = NextModule::None;
        self.current_screen = Screen::Main;
        self.input_state.first_read_flag = true;
        self.options_selected_entry = 0;
    }
}
