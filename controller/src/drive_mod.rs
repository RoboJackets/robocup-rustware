extern crate alloc;

use alloc::string::ToString;
use core::cmp;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::{Baseline, Text},
    Drawable,
};
use ssd1306::{prelude::*, Ssd1306};
use teensy4_pins::t41::*;

pub type Display<'a> = &'a mut Ssd1306<
    I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
>;

pub struct InternalSettngs {
    robot_id: u8,
    team: u8,
    shoot_mode: u8,
    trigger_mode: u8,
    dribbler_speed: u8,
    kick_strength: u8,
    //role: u8
}

pub enum Screen {
    Main = 0,
    MainReceiv = 1,
    Options = 2,
}

struct InternalState {
    dribbler_enabled: bool,
    kicker_state: u8,
    conn_acks: [bool; 10],
    conn_acks_attempts: u32,
    current_screen: Screen,
    options_selected_entry: i8,
    btn_last: u8,
}

pub enum Button {
    Left = 0,
    Right = 1,
    Up = 2,
    Down = 3,
}

pub struct DriveMod {
    settings: InternalSettngs,
    state: InternalState,
}

fn button_rising(old_state: u8, new_state: u8, button: u8) -> bool {
    let mask = 1 << button;
    (old_state & mask) == 0 && (new_state & mask) != 0
}

fn button_pressed(old_state: u8, new_state: u8, button: u8) -> bool {
    let mask = 1 << button;
    (old_state & mask) != 0 && (new_state & mask) != 0
}

const TEAM_NAME_MAP: [&str; 2] = ["BLU", "YLW"];
const SHOOT_MODE_MAP: [&str; 2] = ["KICK", "CHIP"];
const TRIGGER_MODE_MAP: [&str; 3] = ["DISB", "IMM ", "BRKB"];

impl DriveMod {
    pub fn new() -> DriveMod {
        DriveMod {
            settings: InternalSettngs {
                robot_id: 0,
                team: 0,
                shoot_mode: 0,
                trigger_mode: 0,
                dribbler_speed: 0,
                kick_strength: 0,
            },
            state: InternalState {
                dribbler_enabled: false,
                kicker_state: 0,
                conn_acks: [false; 10],
                conn_acks_attempts: 0,
                current_screen: Screen::Main,
                options_selected_entry: 0,
                btn_last: 0,
            },
        }
    }

    pub fn render(&self, display: Display) {
        self.render_common_header(display);
        match self.state.current_screen {
            Screen::Main => self.render_main_screen(display),
            Screen::MainReceiv => self.render_main_receiv(display),
            Screen::Options => self.render_settings(display),
        }
    }

    fn render_common_header(&self, display: Display) {
        //text styles
        //TODO: figure out a way to only do these once
        let default_text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .background_color(BinaryColor::Off)
            .build();

        let selected_text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::Off)
            .background_color(BinaryColor::On)
            .build();

        let team_name = TEAM_NAME_MAP[self.settings.team as usize].to_string();
        let team_name = alloc::fmt::format(format_args!("{}{}", team_name, self.settings.robot_id));

        Text::with_baseline(
            team_name.as_str(),
            Point::new(2, 0),
            default_text_style,
            Baseline::Top,
        )
        .draw(display)
        .unwrap();

        // connection status
        let mut conn_success = 0;
        for i in 0..self.state.conn_acks.len() {
            if self.state.conn_acks[i] {
                conn_success += 1;
            }
        }
        let conn_success_percent =
            conn_success / (cmp::min(cmp::max(self.state.conn_acks_attempts, 1), 10));
        let conn_string = match conn_success_percent {
            0 => "DISC".to_string(),
            1 => "100%".to_string(),
            _ => alloc::fmt::format(format_args!(" {:02}%", conn_success_percent * 100)),
        };
        Text::with_baseline(
            conn_string.as_str(),
            Point::new(104, 0),
            default_text_style,
            Baseline::Top,
        )
        .draw(display)
        .unwrap();

        let screen_name = match self.state.current_screen {
            Screen::Main => "Send Status",
            Screen::MainReceiv => "Robot Status",
            Screen::Options => "Options",
        };
        let screen_name_len = screen_name.len() as i32;
        let width = 6 * screen_name_len;
        let x = (128 - width) / 2;
        Text::with_baseline(
            screen_name,
            Point::new(x, 0),
            selected_text_style,
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
    }

    fn render_main_screen(&self, display: Display) {
        Text::with_baseline(
            "This is the send screen!",
            Point::new(2, 14),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
    }

    fn render_main_receiv(&self, display: Display) {
        Text::with_baseline(
            "This is the receive screen!",
            Point::new(2, 14),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
    }

    fn render_settings(&self, display: Display) {
        //text styles
        //TODO: figure out a way to only do these once
        let default_text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .background_color(BinaryColor::Off)
            .build();

        let selected_text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::Off)
            .background_color(BinaryColor::On)
            .build();

        let entry_names = [
            "Back",
            "Robot ID",
            "Team",
            "Shoot Mode",
            "Trigger Mode",
            "Dribbler Speed",
            "Kick Strength",
        ];
        let entry_values = [
            "",
            &self.settings.robot_id.to_string(),
            &TEAM_NAME_MAP[self.settings.team as usize],
            &SHOOT_MODE_MAP[self.settings.shoot_mode as usize],
            &TRIGGER_MODE_MAP[self.settings.trigger_mode as usize],
            &alloc::fmt::format(format_args!("{}%", self.settings.dribbler_speed)),
            &alloc::fmt::format(format_args!("{}%", self.settings.kick_strength)),
        ];

        let page_size: i32 = 5;
        let start: i32 = cmp::min(
            entry_names.len() as i32 - page_size,
            self.state.options_selected_entry as i32,
        );

        for i in 0..page_size {
            let entry_name = entry_names[(start + i) as usize];
            let entry_value = entry_values[(start + i) as usize];
            let y: i32 = 14 + i * 10;
            let is_selected = start + i == self.state.options_selected_entry as i32;
            Text::with_baseline(
                entry_name,
                Point::new(2, y),
                if is_selected {
                    selected_text_style
                } else {
                    default_text_style
                },
                Baseline::Top,
            )
            .draw(display)
            .unwrap();
            Text::with_baseline(
                entry_value,
                Point::new(104, y),
                if is_selected {
                    selected_text_style
                } else {
                    default_text_style
                },
                Baseline::Top,
            )
            .draw(display)
            .unwrap();
        }
    }

    fn handle_entry_modify(&mut self, increment: bool) {
        match self.state.options_selected_entry {
            0 => {
                if increment {
                    self.state.current_screen = Screen::Main;
                }
            }
            1 => {
                if increment && self.settings.robot_id < 9 {
                    self.settings.robot_id += 1;
                } else if !increment && self.settings.robot_id > 0 {
                    self.settings.robot_id -= 1;
                }
            }
            2 => {
                self.settings.team = if self.settings.team == 0 { 1 } else { 0 };
            }
            3 => {
                self.settings.shoot_mode = if self.settings.shoot_mode == 0 { 1 } else { 0 };
            }
            4 => {
                if increment {
                    if self.settings.trigger_mode == 2 {
                        self.settings.trigger_mode = 0;
                    } else {
                        self.settings.trigger_mode += 1;
                    }
                } else if !increment {
                    if self.settings.trigger_mode == 0 {
                        self.settings.trigger_mode = 2;
                    } else {
                        self.settings.trigger_mode -= 1;
                    }
                }
            }
            5 => {
                if increment && self.settings.dribbler_speed <= 95 {
                    self.settings.dribbler_speed += 5;
                } else if !increment && self.settings.dribbler_speed >= 5 {
                    self.settings.dribbler_speed -= 5;
                }
            }
            6 => {
                if increment && self.settings.kick_strength <= 95 {
                    self.settings.kick_strength += 5;
                } else if !increment && self.settings.kick_strength >= 5 {
                    self.settings.kick_strength -= 5;
                }
            }
            _ => {}
        }
    }

    pub fn update_buttons(&mut self, buttons: u8) {
        //get the buttons difference
        let old_state = self.state.btn_last;
        self.state.btn_last = buttons;

        match self.state.current_screen {
            Screen::Main => {
                if button_rising(old_state, buttons, Button::Right as u8) {
                    self.state.current_screen = Screen::MainReceiv;
                } else if button_rising(old_state, buttons, Button::Up as u8) {
                    self.state.current_screen = Screen::Options;
                }
            }
            Screen::MainReceiv => {
                if button_rising(old_state, buttons, Button::Right as u8) {
                    self.state.current_screen = Screen::Main;
                } else if button_rising(old_state, buttons, Button::Up as u8) {
                    self.state.current_screen = Screen::Options;
                }
            }
            Screen::Options => {
                if button_rising(old_state, buttons, Button::Up as u8) {
                    self.state.options_selected_entry =
                        cmp::max(self.state.options_selected_entry - 1, 0);
                }
                if button_rising(old_state, buttons, Button::Down as u8) {
                    self.state.options_selected_entry =
                        cmp::min(self.state.options_selected_entry + 1, 6);
                }
                if button_rising(old_state, buttons, Button::Right as u8) {
                    self.handle_entry_modify(true);
                }
                if button_rising(old_state, buttons, Button::Left as u8) {
                    self.handle_entry_modify(false);
                }

                //also for button holds
                if button_pressed(old_state, buttons, Button::Right as u8) {
                    self.handle_entry_modify(true);
                }
                if button_pressed(old_state, buttons, Button::Left as u8) {
                    self.handle_entry_modify(false);
                }
            }
        }
    }
}
