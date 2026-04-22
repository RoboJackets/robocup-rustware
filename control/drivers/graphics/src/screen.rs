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
pub trait ScreenDisplay<Color>: DrawTarget<Color = Color> {
    type DisplayError;
    fn init_disp(&mut self) -> Result<(), Self::DisplayError>;
    fn clear_disp(&mut self);
    fn flush_disp(&mut self) -> Result<(), Self::DisplayError>;
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
    robot_id: u8,
    blue_team: bool,
}
impl<'a, D> Screen<'a, D>
where
    D: ScreenDisplay<BinaryColor> + DrawTarget<Color = BinaryColor>,
{
    pub fn set_team(&mut self, blue_team: bool) {
        self.blue_team = blue_team;
        self.data.team = if blue_team { "Blue" } else { "Yellow" };
    }

    pub fn set_robot_id(&mut self, robot_id: u8) {
        self.robot_id = robot_id;
        self.data.robot_id = robot_id as u16;
    }

    /// Initialize a new Screen with defaults for data and errors.
    /// Default state is DisplayState::Start.
    /// Requires robot_id: u16, blue_team: bool, display: D
    pub fn new(robot_id: u8, blue_team: bool, display: D) -> Self {
        let instance = Screen {
            state: DisplayState::Start,
            data: MainWindow::new(robot_id as u16, if blue_team { "Blue" } else { "Yellow" }),
            errors: ErrorScreen::new("Error", "Unspecified error".to_string()),
            display: display,
            robot_id,
            blue_team,
        };
        return instance;
    }

    /// Initializes and clears the display.
    /// Returns Result<(), D::DisplayError>.
    pub fn init_display(&mut self) -> Result<(), D::DisplayError> {
        self.display.init_disp()?;
        self.display.clear_disp();
        Ok(())
    }

    /// Draws to the screen and flushes the queue, depending on the current state.
    /// Does not change the current data to be displayed; only sends the data to the screen.
    pub fn draw(&mut self) -> Result<(), DrawError<D::DisplayError, D::Error>> {
        self.display.clear_disp();
        match self.state {
            DisplayState::Start => {
                StartScreen::new(
                    Point::new(0, 0),
                    Point::new(24, 8),
                    self.blue_team,
                    self.robot_id,
                )
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
            .flush_disp()
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

    fn init_disp(&mut self) -> Result<(), Self::DisplayError> {
        self.init()?;
        self.clear();
        Ok(())
    }

    fn clear_disp(&mut self) {
        self.clear();
    }

    fn flush_disp(&mut self) -> Result<(), Self::DisplayError> {
        self.flush()?;
        Ok(())
    }
}
