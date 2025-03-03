use ssd1306::{prelude::*, Ssd1306};
use teensy4_pins::t41::*;

pub type Display<'a> = &'a mut Ssd1306<
    I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
>;

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

use robojackets_robocup_control::{Delay2, RFRadio, SharedSPI};

pub enum NextModule {
    None = 0,       //don't change
    Menu = 1,       //send back to menu
    Drive_Mode = 2, //directly go to drive mode
}

pub struct RadioSettings {
    pub team: u8,
    pub robot_id: u8,
}

pub trait ControllerModule {
    fn new() -> Self;
    fn update_display(&self, display: Display);
    fn update_inputs(&mut self, input: InputStateUpdate);
    fn radio_update(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2);
    fn update_settings(&mut self, settings: &mut RadioSettings);
    fn next_module(&self) -> NextModule;
}
