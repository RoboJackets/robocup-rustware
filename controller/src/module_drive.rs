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
use robojackets_robocup_control::{Delay2, RFRadio, SharedSPI, CHANNEL};
use rtic_monotonics::{systick::Systick, Monotonic};
use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;
use ssd1306::{prelude::*, Ssd1306};
use teensy4_pins::t41::*;

use robojackets_robocup_rtp::{
    control_message::{ShootMode, TriggerMode},
    ControlMessage, ControlMessageBuilder, RobotStatusMessage, Team, BASE_STATION_ADDRESSES,
    CONTROL_MESSAGE_SIZE, ROBOT_RADIO_ADDRESSES, ROBOT_STATUS_SIZE,
};

use ncomm_utils::packing::Packable;

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
    dribbler_speed: i8,
    kick_strength: u8,
    //role: u8
}

#[derive(PartialEq)]
enum Screen {
    Main = 0,
    MainReceiv = 1,
    Options = 2,
}

struct InputState {
    btn_last: u8,
    btn_press_timeout: u32,

    joy_lx: u16,
    joy_ly: u16,
    joy_rx: u16,
    joy_ry: u16,
}

pub struct InputStateUpdate {
    pub btn_left: Option<bool>,
    pub btn_right: Option<bool>,
    pub btn_up: Option<bool>,
    pub btn_down: Option<bool>,

    pub joy_lx: Option<u16>,
    pub joy_ly: Option<u16>,
    pub joy_rx: Option<u16>,
    pub joy_ry: Option<u16>,
}

struct InternalState {
    dribbler_enabled: bool,
    kicker_state: u8,

    ball_sense_status: bool,
    kick_health: bool,
    battery_voltage: u8,
    motor_error: u8,
    fpga_status: bool,

    conn_acks_last: [bool; 100],
    conn_acks_attempts: u32,

    current_screen: Screen,
    options_selected_entry: i8,

    input_state: InputState,

    pend_radio_config_update: bool,
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

const TEAM_NAME_MAP: [&str; 2] = ["BLU", "YLW"];
const SHOOT_MODE_MAP: [&str; 2] = ["KICK", "CHIP"];
const TRIGGER_MODE_MAP: [&str; 3] = ["DISB", "IMM ", "BRKB"];

fn encode_btn_state(left: bool, right: bool, up: bool, down: bool) -> u8 {
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

fn render_text(display: Display, text: &str, x: u8, y: u8, highlight: bool) {
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

    Text::with_baseline(
        text,
        Point::new(x as i32, y as i32),
        if highlight {
            selected_text_style
        } else {
            default_text_style
        },
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
}

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

                conn_acks_last: [false; 100],
                conn_acks_attempts: 0,

                current_screen: Screen::Main,
                options_selected_entry: 0,

                input_state: InputState {
                    btn_last: 0,
                    btn_press_timeout: 0,

                    joy_lx: 0,
                    joy_ly: 0,
                    joy_rx: 0,
                    joy_ry: 0,
                },

                pend_radio_config_update: false,
            },
        }
    }

    fn get_successful_ack_count(&self) -> u32 {
        let mut count = 0;
        for i in 0..cmp::min(100, self.state.conn_acks_attempts) {
            if self.state.conn_acks_last[i as usize] {
                count += 1;
            }
        }
        return count;
    }

    fn render_header(&self, display: Display) {
        let team_name = TEAM_NAME_MAP[self.settings.team as usize].to_string();
        let team_name = alloc::fmt::format(format_args!("{}{}", team_name, self.settings.robot_id));
        render_text(display, &team_name, 2, 0, false);

        let ack_count = self.get_successful_ack_count() as f32;
        let ack_percent = (ack_count * 100.0)
            / (cmp::min(cmp::max(self.state.conn_acks_attempts, 1), 100)) as f32;

        let ack_percent = ack_percent - (ack_percent % 1.0);

        let ack_str = alloc::fmt::format(format_args!("{:02}%", ack_percent));
        render_text(display, &ack_str, 104, 0, false);

        let screen_name = match self.state.current_screen {
            Screen::Main => "Send Status",
            Screen::MainReceiv => "Robot Status",
            Screen::Options => "Options",
        };
        let screen_name_len = screen_name.len() as i32;
        let width = 6 * screen_name_len;
        let x = (128 - width) / 2;
        render_text(display, screen_name, x as u8, 0, true);
    }

    fn render_main_screen(&self, display: Display) {
        //joysticks
        let joy_lx_str = alloc::fmt::format(format_args!("LX: {}", self.state.input_state.joy_lx));
        let joy_ly_str = alloc::fmt::format(format_args!("LY: {}", self.state.input_state.joy_ly));
        let joy_rx_str = alloc::fmt::format(format_args!("RX: {}", self.state.input_state.joy_rx));
        let joy_ry_str = alloc::fmt::format(format_args!("RY: {}", self.state.input_state.joy_ry));

        render_text(display, joy_lx_str.as_str(), 2, 14, false);
        render_text(display, joy_ly_str.as_str(), 2, 24, false);
        render_text(display, joy_rx_str.as_str(), 64, 14, false);
        render_text(display, joy_ry_str.as_str(), 64, 24, false);

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

        render_text(display, dribbler_str, 2, 34, false);
        render_text(display, kicker_str, 64, 34, false);
    }

    fn render_main_receiv(&self, display: Display) {
        let conn_success = self.get_successful_ack_count();
        //if we've gotten no connections at all, whatever we put on the screen will be lies
        if conn_success == 0 {
            render_text(display, "No Status to Display", 2, 14, false);
            render_text(display, "No Connection", 2, 24, false);
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
            let y = 14 + i * 10;

            render_text(display, entry_name, 4, y as u8, false);
            render_text(display, entry_value, 104, y as u8, false);
        }
    }

    fn render_settings(&self, display: Display) {
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
            render_text(display, entry_name, 2, y as u8, is_selected);
            render_text(display, entry_value, 104, y as u8, is_selected);
        }
    }

    pub fn update_display(&self, display: Display) {
        self.render_header(display);
        match self.state.current_screen {
            Screen::Main => self.render_main_screen(display),
            Screen::MainReceiv => self.render_main_receiv(display),
            Screen::Options => self.render_settings(display),
        }
    }

    fn handle_entry_modify(&mut self, increment: bool) {
        match self.state.options_selected_entry {
            0 => {
                if increment {
                    self.state.current_screen = Screen::Main;
                    self.state.pend_radio_config_update = true;
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
                for i in 0..self.state.conn_acks_last.len() {
                    self.state.conn_acks_last[i] = false;
                }
            }
            2 => {
                self.settings.team = if self.settings.team == 0 { 1 } else { 0 };

                //new team, wipe the connection status
                self.state.conn_acks_attempts = 0;
                for i in 0..self.state.conn_acks_last.len() {
                    self.state.conn_acks_last[i] = false;
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

    fn btn_rising(&mut self, old_state: u8, new_state: u8, button: Button) -> bool {
        let mask = 1 << button as u8;
        //This is ~500 ms
        if (old_state & mask) == 0 && (new_state & mask) != 0 {
            self.state.input_state.btn_press_timeout = Systick::now().ticks() + 500;
            return true;
        } else {
            return false;
        }
    }

    fn btn_held(&mut self, old_state: u8, new_state: u8, button: Button) -> bool {
        let mask = 1 << button as u8;
        if Systick::now().ticks() < self.state.input_state.btn_press_timeout {
            return false;
        }
        self.state.input_state.btn_press_timeout = 0;
        (old_state & mask) != 0 && (new_state & mask) != 0
    }

    fn update_joysticks(&mut self, lx: u16, ly: u16, rx: u16, ry: u16) {
        self.state.input_state.joy_lx = lx;
        self.state.input_state.joy_ly = ly;
        self.state.input_state.joy_rx = rx;
        self.state.input_state.joy_ry = ry;
    }

    fn update_buttons(&mut self, buttons: u8) {
        //get the buttons difference
        let old_state = self.state.input_state.btn_last;
        self.state.input_state.btn_last = buttons;

        match self.state.current_screen {
            Screen::Main => {
                if self.btn_rising(old_state, buttons, Button::Right) {
                    self.state.current_screen = Screen::MainReceiv;
                } else if self.btn_rising(old_state, buttons, Button::Up) {
                    self.state.current_screen = Screen::Options;
                } else if self.btn_rising(old_state, buttons, Button::Left) {
                    self.state.dribbler_enabled = !self.state.dribbler_enabled;
                } else if self.btn_rising(old_state, buttons, Button::Down) {
                    self.state.kicker_state = if self.state.kicker_state == 0 { 1 } else { 0 };
                }
            }
            Screen::MainReceiv => {
                if self.btn_rising(old_state, buttons, Button::Right) {
                    self.state.current_screen = Screen::Main;
                } else if self.btn_rising(old_state, buttons, Button::Up) {
                    self.state.current_screen = Screen::Options;
                }
            }
            Screen::Options => {
                if self.btn_rising(old_state, buttons, Button::Up) {
                    self.state.options_selected_entry =
                        cmp::max(self.state.options_selected_entry - 1, 0);
                }
                if self.btn_rising(old_state, buttons, Button::Down) {
                    self.state.options_selected_entry =
                        cmp::min(self.state.options_selected_entry + 1, 6);
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

    pub fn update_inputs(&mut self, inputs: InputStateUpdate) {
        //only update joysticks if all are present
        if inputs.joy_lx.is_some()
            && inputs.joy_ly.is_some()
            && inputs.joy_rx.is_some()
            && inputs.joy_ry.is_some()
        {
            self.update_joysticks(
                inputs.joy_lx.unwrap(),
                inputs.joy_ly.unwrap(),
                inputs.joy_rx.unwrap(),
                inputs.joy_ry.unwrap(),
            );
        }

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
            self.update_buttons(new_state);
        }
    }

    fn generate_outgoing_packet(&mut self) -> ControlMessage {
        //this is a *very* basic joystick mapping
        let lx = self.state.input_state.joy_lx;
        let ly = self.state.input_state.joy_ly;
        let lw = self.state.input_state.joy_rx as f32 / 1024.0 - 0.5;

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

    fn enable_radio_listen(
        &mut self,
        radio: &mut RFRadio,
        spi: &mut SharedSPI,
        delay: &mut Delay2,
    ) {
        radio.set_payload_size(ROBOT_STATUS_SIZE as u8, spi, delay);
        radio.start_listening(spi, delay);
    }

    fn disable_radio_listen(
        &mut self,
        radio: &mut RFRadio,
        spi: &mut SharedSPI,
        delay: &mut Delay2,
    ) {
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, delay);
        radio.stop_listening(spi, delay);
    }

    fn configure_radio(
        &mut self,
        radio: &mut RFRadio,
        spi: &mut SharedSPI,
        delay: &mut Delay2,
    ) {
        log::info!("Configuring Radio");

        radio.set_pa_level(PowerAmplifier::PALow, spi, delay);
        radio.set_channel(CHANNEL, spi, delay);
        radio.open_writing_pipe(
            ROBOT_RADIO_ADDRESSES[self.settings.team as usize][self.settings.robot_id as usize],
            spi,
            delay,
        );
        radio.open_reading_pipe(1, BASE_STATION_ADDRESSES[0], spi, delay);
        self.enable_radio_listen(radio, spi, delay);
    }

    pub fn radio_update(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2) {
        if self.state.current_screen == Screen::Options {
            return;
        }

        //poll for messages we got in the meantime
        self.radio_poll(radio, spi, delay);

        //update radio config if needed
        if self.state.pend_radio_config_update {
            self.configure_radio(radio, spi, delay);
            self.state.pend_radio_config_update = false;
        }

        self.disable_radio_listen(radio, spi, delay);

        let control_message = self.generate_outgoing_packet();

        let mut packed_data = [0u8; CONTROL_MESSAGE_SIZE];
        control_message.pack(&mut packed_data).unwrap();

        let report = radio.write(&packed_data, spi, delay);
        radio.flush_tx(spi, delay);

        self.enable_radio_listen(radio, spi, delay);

        match report {
            true => {
                log::info!("Data Sent!");
            }
            false => {
                log::info!("Unable to Send Data!");
            }
        }

        //update the connection status
        for i in (1..(self.state.conn_acks_last.len() - 1)).rev() {
            self.state.conn_acks_last[i] = self.state.conn_acks_last[i - 1];
        }

        self.state.conn_acks_last[0] = report;

        self.state.conn_acks_attempts += 1;
    }

    fn update_incoming_packet(&mut self, msg: RobotStatusMessage) {
        self.state.ball_sense_status = msg.ball_sense_status;
        self.state.kick_health = msg.kick_healthy;
        self.state.battery_voltage = msg.battery_voltage;
        self.state.motor_error = msg.motor_errors;
        self.state.fpga_status = msg.fpga_status;
    }

    fn radio_poll(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2) {
        if self.state.current_screen == Screen::Options {
            return;
        }

        if radio.packet_ready(spi, delay) {
            let mut data = [0u8; ROBOT_STATUS_SIZE];
            radio.read(&mut data, spi, delay);

            match RobotStatusMessage::unpack(&data[..]) {
                Ok(data) => {
                    log::info!("Data Received!");
                    self.update_incoming_packet(data);
                }
                Err(err) => {
                    log::info!("Unable to Unpack Data: {:?}", err)
                }
            }
        }
    }
}
