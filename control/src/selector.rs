//!
//! Selector for the controller
//! 

use imxrt_hal::gpio::Input;
use robojackets_robocup_rtp::Team;
use imxrt_iomuxc::{Config, PullKeeper};


pub const PIN_CONFIG: Config = Config::modify().set_pull_keeper(Some(PullKeeper::Pullup47k));


/// Get the team and id of the robot based on the selector switch
pub fn get_team_and_id<P0, P1, P2, P3>(
    p0: &Input<P0>,
    p1: &Input<P1>,
    p2: &Input<P2>,
    p3: &Input<P3>,
) -> (Team, u8) {
    match (p0.is_set(), p1.is_set(), p2.is_set(), p3.is_set()) {
        (false, false, false, false) => (Team::Yellow, 0),
        (false, false, false, true) => (Team::Yellow, 1),
        (false, false, true, false) => (Team::Yellow, 2),
        (false, false, true, true) => (Team::Yellow, 3),
        (false, true, false, false) => (Team::Yellow, 4),
        (false, true, false, true) => (Team::Yellow, 5),
        (false, true, true, false) => (Team::Yellow, 6),
        (false, true, true, true) => (Team::Yellow, 7),
        (true, false, false, false) => (Team::Blue, 0),
        (true, false, false, true) => (Team::Blue, 1),
        (true, false, true, false) => (Team::Blue, 2),
        (true, false, true, true) => (Team::Blue, 3),
        (true, true, false, false) => (Team::Blue, 4),
        (true, true, false, true) => (Team::Blue, 5),
        (true, true, true, false) => (Team::Blue, 6),
        (true, true, true, true) => (Team::Blue, 7),
    }
}
