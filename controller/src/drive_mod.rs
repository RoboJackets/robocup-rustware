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
use rtic_monotonics::{
    systick::{
        fugit::{Duration, Instant},
        Systick,
    },
    Monotonic,
};
use ssd1306::{prelude::*, Ssd1306};
use teensy4_pins::t41::*;

use robojackets_robocup_rtp::{
    control_message::{ShootMode, TriggerMode},
    ControlMessage, ControlMessageBuilder, RobotStatusMessage, Team, CONTROL_MESSAGE_SIZE,
};

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

    ball_sense_status: bool,
    kick_health: bool,
    battery_voltage: u8,
    motor_error: u8,
    fpga_status: bool,

    conn_acks: [bool; 10],
    conn_acks_attempts: u32,

    current_screen: Screen,
    options_selected_entry: i8,

    btn_last: u8,
    btn_press_timeout: u32,

    joy_lx: u16,
    joy_ly: u16,
    joy_rx: u16,
    joy_ry: u16,
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

pub fn encode_btn_state(left: bool, right: bool, up: bool, down: bool) -> u8 {
    let mut btn = 0u8;
    if left {
        btn |= 1 << 0;
    }
    if right {
        btn |= 1 << 1;
    }
    if up {
        btn |= 1 << 2;
    }
    if down {
        btn |= 1 << 3;
    }
    return btn;
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

                ball_sense_status: false,
                kick_health: false,
                battery_voltage: 0,
                motor_error: 0,
                fpga_status: false,

                conn_acks: [false; 10],
                conn_acks_attempts: 0,

                current_screen: Screen::Main,
                options_selected_entry: 0,

                btn_last: 0,
                btn_press_timeout: 0,
                joy_lx: 512,
                joy_ly: 512,
                joy_rx: 512,
                joy_ry: 512,
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
            conn_success as f32 / (cmp::min(cmp::max(self.state.conn_acks_attempts, 1), 10)) as f32;
        let conn_string = match conn_success_percent {
            0.0 => "DISC".to_string(),
            1.0 => "100%".to_string(),
            _ => alloc::fmt::format(format_args!(" {:02}%", conn_success_percent * 100.0)),
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
        //joysticks
        let joy_lx_str = alloc::fmt::format(format_args!("LX: {}", self.state.joy_lx));
        let joy_ly_str = alloc::fmt::format(format_args!("LY: {}", self.state.joy_ly));
        let joy_rx_str = alloc::fmt::format(format_args!("RX: {}", self.state.joy_rx));
        let joy_ry_str = alloc::fmt::format(format_args!("RY: {}", self.state.joy_ry));

        Text::with_baseline(
            joy_lx_str.as_str(),
            Point::new(2, 14),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
        Text::with_baseline(
            joy_ly_str.as_str(),
            Point::new(2, 24),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
        Text::with_baseline(
            joy_rx_str.as_str(),
            Point::new(64, 14),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
        Text::with_baseline(
            joy_ry_str.as_str(),
            Point::new(64, 24),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();

        //dribbler and kicker
        let dribbler_str = if self.state.dribbler_enabled {
            "Drib ON"
        } else {
            "Drib OFF"
        };
        let kicker_str = if self.settings.trigger_mode != 0 {
            match self.state.kicker_state {
                1 => "Kick ON",
                0 => "Kick OFF",
                _ => "Kick ERR",
            }
        } else {
            "Kick DISB"
        };
        Text::with_baseline(
            dribbler_str,
            Point::new(2, 34),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
        Text::with_baseline(
            kicker_str,
            Point::new(64, 34),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
    }

    fn render_main_receiv(&self, display: Display) {
        let mut conn_success = 0;
        for i in 0..self.state.conn_acks.len() {
            if self.state.conn_acks[i] {
                conn_success += 1;
            }
        }

        //if we've gotten no connections at all, whatever we put on the screen will be lies
        if conn_success == 0 {
            Text::with_baseline(
                "No Status to Display\nNo Connection",
                Point::new(2, 14),
                MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                Baseline::Top,
            )
            .draw(display)
            .unwrap();
            return;
        }

        let entry_names = ["Ball Sense", "Kick Health", "Battery", "Motor Err", "FPGA"];
        let entry_values = [
            if self.state.ball_sense_status {
                "ON"
            } else {
                "OFF"
            },
            if self.state.kick_health { "OK" } else { "ERR" },
            &alloc::fmt::format(format_args!("{}", self.state.battery_voltage)),
            &alloc::fmt::format(format_args!("{}", self.state.motor_error)),
            if self.state.fpga_status { "OK" } else { "ERR" },
        ];

        for i in 0..5 {
            let entry_name = entry_names[i];
            let entry_value = entry_values[i];
            let y: i32 = 14 + i as i32 * 10;
            Text::with_baseline(
                entry_name,
                Point::new(2, y),
                MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                Baseline::Top,
            )
            .draw(display)
            .unwrap();
            Text::with_baseline(
                entry_value,
                Point::new(104, y),
                MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
                Baseline::Top,
            )
            .draw(display)
            .unwrap();
        }
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

                //new robot, wipe the connection status
                self.state.conn_acks_attempts = 0;
                for i in 0..self.state.conn_acks.len() {
                    self.state.conn_acks[i] = false;
                }
            }
            2 => {
                self.settings.team = if self.settings.team == 0 { 1 } else { 0 };

                //new team, wipe the connection status
                self.state.conn_acks_attempts = 0;
                for i in 0..self.state.conn_acks.len() {
                    self.state.conn_acks[i] = false;
                }
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

    fn button_rising(&mut self, old_state: u8, new_state: u8, button: u8) -> bool {
        let mask = 1 << button;
        //This is ~500 ms
        if (old_state & mask) == 0 && (new_state & mask) != 0 {
            self.state.btn_press_timeout = Systick::now().ticks() + 500;
            return true;
        } else {
            return false;
        }
    }

    fn button_pressed(&mut self, old_state: u8, new_state: u8, button: u8) -> bool {
        let mask = 1 << button;
        if Systick::now().ticks() < self.state.btn_press_timeout {
            return false;
        }
        self.state.btn_press_timeout = 0;
        (old_state & mask) != 0 && (new_state & mask) != 0
    }

    pub fn update_buttons(&mut self, buttons: u8) {
        //get the buttons difference
        let old_state = self.state.btn_last;
        self.state.btn_last = buttons;

        match self.state.current_screen {
            Screen::Main => {
                if self.button_rising(old_state, buttons, Button::Right as u8) {
                    self.state.current_screen = Screen::MainReceiv;
                } else if self.button_rising(old_state, buttons, Button::Up as u8) {
                    self.state.current_screen = Screen::Options;
                } else if self.button_rising(old_state, buttons, Button::Left as u8) {
                    self.state.dribbler_enabled = !self.state.dribbler_enabled;
                } else if self.button_rising(old_state, buttons, Button::Down as u8) {
                    self.state.kicker_state = if self.state.kicker_state == 0 { 1 } else { 0 };
                }
            }
            Screen::MainReceiv => {
                if self.button_rising(old_state, buttons, Button::Right as u8) {
                    self.state.current_screen = Screen::Main;
                } else if self.button_rising(old_state, buttons, Button::Up as u8) {
                    self.state.current_screen = Screen::Options;
                }
            }
            Screen::Options => {
                if self.button_rising(old_state, buttons, Button::Up as u8) {
                    self.state.options_selected_entry =
                        cmp::max(self.state.options_selected_entry - 1, 0);
                }
                if self.button_rising(old_state, buttons, Button::Down as u8) {
                    self.state.options_selected_entry =
                        cmp::min(self.state.options_selected_entry + 1, 6);
                }
                if self.button_rising(old_state, buttons, Button::Right as u8) {
                    self.handle_entry_modify(true);
                } else if self.button_pressed(old_state, buttons, Button::Right as u8) {
                    self.handle_entry_modify(true);
                }

                if self.button_rising(old_state, buttons, Button::Left as u8) {
                    self.handle_entry_modify(false);
                } else if self.button_pressed(old_state, buttons, Button::Left as u8) {
                    self.handle_entry_modify(false);
                }
            }
        }
    }

    pub fn update_joysticks(&mut self, lx: u16, ly: u16, rx: u16, ry: u16) {
        self.state.joy_lx = lx;
        self.state.joy_ly = ly;
        self.state.joy_rx = rx;
        self.state.joy_ry = ry;
    }

    pub fn generate_outgoing_packet(&mut self) -> ControlMessage {
        //this is a *very* basic joystick mapping
        let lx = self.state.joy_lx;
        let ly = self.state.joy_ly;
        let lw = self.state.joy_rx as i16 - 512;

        let msg = ControlMessageBuilder::new()
            .robot_id(self.settings.robot_id)
            .team(if self.settings.team == 0 {
                Team::Blue
            } else {
                Team::Yellow
            })
            .shoot_mode(match self.settings.shoot_mode {
                0 => ShootMode::Kick,
                1 => ShootMode::Chip,
                _ => ShootMode::Kick,
            })
            .trigger_mode(match self.settings.trigger_mode {
                0 => TriggerMode::StandDown,
                1 => TriggerMode::Immediate,
                2 => TriggerMode::OnBreakBeam,
                _ => TriggerMode::StandDown,
            })
            .body_x(lx as f32)
            .body_y(ly as f32)
            .body_w(lw as f32)
            .dribbler_speed(self.settings.dribbler_speed as i8)
            .kick_strength(self.settings.kick_strength)
            .build();

        return msg;
    }

    pub fn report_send_result(&mut self, success: bool) {
        //shift the array
        for i in 0..self.state.conn_acks.len() - 1 {
            self.state.conn_acks[i] = self.state.conn_acks[i + 1];
        }
        self.state.conn_acks[9] = success;

        self.state.conn_acks_attempts += 1;
    }

    pub fn update_incoming_packet(&mut self, msg: RobotStatusMessage) {
        self.state.ball_sense_status = msg.ball_sense_status;
        self.state.kick_health = msg.kick_healthy;
        self.state.battery_voltage = msg.battery_voltage;
        self.state.motor_error = msg.motor_errors;
        self.state.fpga_status = msg.fpga_status;
    }
}
