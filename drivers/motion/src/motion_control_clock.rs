//!
//! Motion Control Clock trait
//!
//! At the time of writing this, the embedded_hal did not have a great
//! interface for timestamp clocks, so this trait is to supplement for now.
//!

/// Clock used for motion control
pub trait MotionControlClock {
    /// Get the elapsed time from the last call too get_elapsed_time_us
    fn get_elapsed_time_us(&mut self) -> u32;
}