use {crate::startup_screen::StartScreen,
    crate::error_screen::ErrorScreen,
    crate::main_window::MainWindow};
use embedded_graphics::{
    prelude::*,
    pixelcolor::BinaryColor,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DisplayState {
    Start,
    Error,
    MainLoop,
}

pub struct Screen<'a, D>
where
    D: DrawTarget<Color = BinaryColor> {
    state: DisplayState,
    data: MainWindow<'a>,
    errors: ErrorScreen<'a>,
    display: &'a mut D,
}
impl<'a, D> Screen<'a, D>
where 
    D: DrawTarget<Color = BinaryColor> {
    pub fn new(robot_id: u16, team: &'a str, display: &'a mut D) -> Self {
        let instance = Screen {
            state: DisplayState::Start, 
            data: MainWindow::new(robot_id, team), 
            errors: ErrorScreen::new("Error", "Unspecified error"), 
            display: display,
        };
        return instance;
    }

    pub fn draw(&mut self) -> Result<(), D::Error> {
        match self.state {
            DisplayState::Start => {
                StartScreen::new(Point::new(0, 0), Point::new(24, 8))
                    .draw(self.display)?;
            }
            DisplayState::MainLoop => {
                self.data.draw(self.display)?;
            }
            DisplayState::Error => {
                self.errors.draw(self.display)?;
            }
        }
        Ok(())
    }

    pub fn main_loop_update(&mut self, battery_percent: u32, kicker_charged: bool, ball_sense: bool, latency: u32) {
        self.data.battery_percent = battery_percent;
        self.data.kicker_charged = kicker_charged;
        self.data.ball_sense = ball_sense;
        self.data.latency = latency;
        self.state = DisplayState::MainLoop;
    }

    pub fn error_update(&mut self, heading: &'a str, message: &'a str) {
        self.errors.update(heading, message);
        self.state = DisplayState::Error;
    }

}