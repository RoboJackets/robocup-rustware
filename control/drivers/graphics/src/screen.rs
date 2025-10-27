use alloc::string::ToString;
use ssd1306::mode::{BufferedGraphicsMode, DisplayConfig};
use ssd1306::prelude::I2CInterface;
use ssd1306::size::DisplaySize128x64;
use ssd1306::Ssd1306;

use {crate::startup_screen::StartScreen,
    crate::error_screen::ErrorScreen,
    crate::main_window::MainWindow};
use embedded_graphics::{
    prelude::*,
    pixelcolor::BinaryColor,
};

/// States for the onboard display that determine what is shown on screen.
/// Start state shows only the start screen by default.
/// Error state shows a title and description of any error.
/// MainLoop state shows a list of important information about the current status of the robot.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DisplayState {
    Start,
    Error,
    MainLoop,
}

/// Trait for a Screen that extends from DrawTarget.
/// initDisp must initialize the display and return a Result<(), Self::DisplayError>
/// clearDisp must remove all contents from the display
/// flushDisp must send to the display all data currently in the queue if it exists.
///     Returns Result<(), Self::DisplayError>
pub trait ScreenDisplay<Color>: DrawTarget<Color=Color> {
    type DisplayError;
    fn initDisp(&mut self) -> Result<(), Self::DisplayError>;
    fn clearDisp(&mut self);
    fn flushDisp(&mut self) -> Result<(), Self::DisplayError>;
}

/// Container for errors in sending data to the display and errors in the display interface.
/// DrawError::Draw maps errors in sending data, such as the draw() command for Drawables.
/// DrawError::DisplayInterface maps errors in commands that act on a ScreenDisplay.
pub enum DrawError<D1, D2> {
    DisplayInterface(D1),
    Draw(D2),
}

/// Struct representing the onboard display for a robot and a state machine for that display.
/// Implemented where the display has trait ScreenDisplay.
/// If the current state is MainLoop, displays current contents of data whenever draw() is called.
/// If the current state is Error, displays errors whenever draw() is called.
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
    /// Initialize a new Screen with defaults for data and errors.
    /// Default state is DisplayState::Start.
    /// Requires robot_id: u16, team: &'a str, display: D
    pub fn new(robot_id: u16, team: &'a str, display: D) -> Self {
        let instance = Screen {
            state: DisplayState::Start,
            data: MainWindow::new(robot_id, team),
            errors: ErrorScreen::new("Error", "Unspecified error".to_string()),
            display: display,
        };
        return instance;
    }

    /// Initializes and clears the display.
    /// Returns Result<(), D::DisplayError>.
    pub fn init_display(&mut self) -> Result<(), D::DisplayError> {
        self.display.initDisp()?;
        self.display.clearDisp();
        Ok(())
    }

    /// Draws to the screen and flushes the queue, depending on the current state.
    /// Does not change the current data to be displayed; only sends the data to the screen.
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

    /// Sets current state to DisplayState::MainLoop and sets the current data to the input params.
    /// battery_percent: u32; the current battery percentage.
    /// kicker_charged: bool; whether or not the kicker is currently ready.
    /// ball_sense: bool; whether or not the robot has the ball.
    /// latency: u32; the radio latency. Currently reserved for future use.
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

    /// Sets the current state to DisplayState::Error and sets the current data to the input params.
    /// heading: &'a str; the heading used to describe the error type.
    /// message: alloc::string::String; detailed description of the error.
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
