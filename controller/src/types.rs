use alloc::boxed::Box;
use ssd1306::{prelude::*, Ssd1306};
use teensy4_pins::t41::*;

pub type Display<'a> = &'a mut Ssd1306<
    I2CInterface<imxrt_hal::lpi2c::Lpi2c<imxrt_hal::lpi2c::Pins<P19, P18>, 1>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
>;

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

/* Adding a new module: 
 *
 * 1. Create a new module file in the src directory. Naming scheme module_<name>.rs
 * 2. Implement the ControllerModule trait for the new module
 * 3. Add the new module to the NextModule enum
 * 4. Add the new module to the MODULE_ENTRIES array
 * 5. Increment MODULE_COUNT
 *
 * Considerations:
 * - There are a few utilities in util.rs that may be useful for new modules.
 * - Prefer to use render_status_title and render_text for rendering text to the display
 * - next_module should return NextModule::None if the module should not change. Do not simply return the id of the current module.
 *   This will cause the module to be reset every time the dispatcher attempts to swap.
 */

#[derive(PartialEq, Clone, Copy)]
pub enum NextModule {
    None = -1,     //don't change
    Menu = 0,      //swap to the menu
    DriveMode = 1, //swap to the drive mode
    RadioMode = 2, //swap to radio test mode
}

pub const TEAM_NAME_MAP: [&str; 2] = ["BLU", "YLW"];

pub struct ModuleEntry {
    pub name: &'static str,
    pub id: NextModule,
}

pub const MODULE_COUNT: usize = 2;

pub type ModuleArr = [Box<dyn ControllerModule>; MODULE_COUNT];

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

pub trait ControllerModule: Send + Sync {
    //perform any nessesary work to update the display
    fn update_display(&self, display: Display);

    //triggered on button press or poll. provides the current state of the inputs.
    fn update_inputs(&mut self, input: InputStateUpdate);

    //perform nessesary work with the radio
    fn radio_update(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2);

    //update the radio settings of the module
    fn update_settings(&mut self, settings: &mut RadioState);

    //poll to see what module to switch to
    fn next_module(&mut self) -> NextModule;

    //triggered when the module is switched to
    fn reset(&mut self);
}
