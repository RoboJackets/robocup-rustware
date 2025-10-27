use alloc::string::ToString;
use ssd1306::Ssd1306;
use ssd1306::mode::{BufferedGraphicsMode, DisplayConfig};
use ssd1306::prelude::I2CInterface;
use ssd1306::size::DisplaySize128x64;

use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use {
    crate::error_screen::ErrorScreen, crate::main_window::MainWindow,
    crate::startup_screen::StartScreen,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DisplayState {
    Start,
    Error,
    MainLoop,
}

pub trait ScreenDisplay<Color>: DrawTarget<Color = Color> {
    type DisplayError;
    fn initDisp(&mut self) -> Result<(), Self::DisplayError>;
    fn clearDisp(&mut self);
    fn flushDisp(&mut self) -> Result<(), Self::DisplayError>;
}

pub enum DrawError<D1, D2> {
    DisplayInterface(D1),
    Draw(D2),
}

pub struct Screen<'a, D>
where
    D: ScreenDisplay<BinaryColor>,
{
    state: DisplayState,
    data: MainWindow<'a>,
    errors: ErrorScreen<'a>,
    display: D,
}
impl<'a, D> Screen<'a, D>
where
    D: ScreenDisplay<BinaryColor> + DrawTarget<Color = BinaryColor>,
{
    pub fn new(robot_id: u16, team: &'a str, display: D) -> Self {
        let instance = Screen {
            state: DisplayState::Start,
            data: MainWindow::new(robot_id, team),
            errors: ErrorScreen::new("Error", "Unspecified error".to_string()),
            display: display,
        };
        return instance;
    }

    pub fn init_display(&mut self) -> Result<(), D::DisplayError> {
        self.display.initDisp()?;
        self.display.clearDisp();
        Ok(())
    }

    pub fn draw(&mut self) -> Result<(), DrawError<D::DisplayError, D::Error>> {
        self.display.clearDisp();
        match self.state {
            DisplayState::Start => {
                StartScreen::new(Point::new(0, 0), Point::new(24, 8))
                    .draw(&mut self.display)
                    .map_err(DrawError::Draw)?;
            }
            DisplayState::MainLoop => {
                self.data.draw(&mut self.display).map_err(DrawError::Draw)?;
            }
            DisplayState::Error => {
                self.errors
                    .draw(&mut self.display)
                    .map_err(DrawError::Draw)?;
            }
        }
        self.display
            .flushDisp()
            .map_err(DrawError::DisplayInterface)?;
        Ok(())
    }

    pub fn main_loop_update(
        &mut self,
        battery_percent: u32,
        kicker_charged: bool,
        ball_sense: bool,
        latency: u32,
    ) {
        self.data.battery_percent = battery_percent;
        self.data.kicker_charged = kicker_charged;
        self.data.ball_sense = ball_sense;
        self.data.latency = latency;
        self.state = DisplayState::MainLoop;
    }

    pub fn error_update(&mut self, heading: &'a str, message: alloc::string::String) {
        self.errors.update(heading, message);
        self.state = DisplayState::Error;
    }
}

impl<B: embedded_hal::blocking::i2c::Write> ScreenDisplay<BinaryColor>
    for Ssd1306<I2CInterface<B>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>
{
    type DisplayError = display_interface::DisplayError;

    fn initDisp(&mut self) -> Result<(), Self::DisplayError> {
        self.init()?;
        self.clear();
        Ok(())
    }

    fn clearDisp(&mut self) {
        self.clear();
    }

    fn flushDisp(&mut self) -> Result<(), Self::DisplayError> {
        self.flush()?;
        Ok(())
    }
}
