use alloc::boxed::Box;
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

#[derive(PartialEq, Clone, Copy)]
pub enum NextModule {
    None = -1,     //don't change
    Menu = 0,      //send back to menu
    DriveMode = 1, //directly go to drive mode
}

pub const TEAM_NAME_MAP: [&str; 2] = ["BLU", "YLW"];

pub const MODULE_COUNT: usize = 2;

pub type ModuleArr = [Box<dyn ControllerModule>; MODULE_COUNT];

pub struct ModuleEntry {
    pub name: &'static str,
    pub id: NextModule,
}

pub const MODULE_ENTRIES: [ModuleEntry; MODULE_COUNT] = [
    ModuleEntry {
        name: "Menu",
        id: NextModule::Menu,
    },
    ModuleEntry {
        name: "Drive Mode",
        id: NextModule::DriveMode,
    },
];

pub enum Button {
    Left = 0,
    Right = 1,
    Up = 2,
    Down = 3,
}

pub struct RadioState {
    pub team: u8,
    pub robot_id: u8,
    pub conn_acks_results: [bool; 100],
    pub conn_acks_attempts: u16,
}

pub trait ControllerModule: Send + Sync {
    fn update_display(&self, display: Display);
    fn update_inputs(&mut self, input: InputStateUpdate);
    fn radio_update(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2);
    fn update_settings(&mut self, settings: &mut RadioState);
    fn next_module(&mut self) -> NextModule;
}
