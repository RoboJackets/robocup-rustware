//!
//! Selector for the controller
//!

use imxrt_hal::gpio::Input;
use imxrt_iomuxc::{Config, PullKeeper};
use robojackets_robocup_rtp::Team;

pub const PIN_CONFIG: Config = Config::modify().set_pull_keeper(Some(PullKeeper::Pullup47k));

/// Get the team and id of the robot based on the selector switch
///
/// Looking at the control board with display towards you and the teensy away from
/// you the selector switch works as follows:
///
/// ----------------------
/// | Yellow | 4 | 2 | 1 |
/// |--------|---|---|---|
/// | Blue   | 0 | 0 | 0 |
/// ----------------------
///
/// Basically, the left-most switch determines the team color with up being
/// yellow and down being blue.  Then the next three switches determine the
/// id of the robot can be read as binary with down being 0 and up being 1.
///
/// For example a selector in this position will be a blue team robot with id 5
/// (4 + 0 + 1).
///
/// -----------------
/// | _ | ^ | _ | ^ |
/// -----------------
///
/// Where _ is down and ^ is up.
pub fn get_team_and_id<P0, P1, P2, P3>(
    p0: &Input<P0>,
    p1: &Input<P1>,
    p2: &Input<P2>,
    p3: &Input<P3>,
) -> (Team, u8) {
    match (p0.is_set(), p1.is_set(), p2.is_set(), p3.is_set()) {
        (false, false, false, false) => (Team::Yellow, 7),
        (false, false, false, true) => (Team::Yellow, 6),
        (false, false, true, false) => (Team::Yellow, 5),
        (false, false, true, true) => (Team::Yellow, 4),
        (false, true, false, false) => (Team::Yellow, 3),
        (false, true, false, true) => (Team::Yellow, 2),
        (false, true, true, false) => (Team::Yellow, 1),
        (false, true, true, true) => (Team::Yellow, 0),
        (true, false, false, false) => (Team::Blue, 7),
        (true, false, false, true) => (Team::Blue, 6),
        (true, false, true, false) => (Team::Blue, 5),
        (true, false, true, true) => (Team::Blue, 4),
        (true, true, false, false) => (Team::Blue, 3),
        (true, true, false, true) => (Team::Blue, 2),
        (true, true, true, false) => (Team::Blue, 1),
        (true, true, true, true) => (Team::Blue, 0),
    }
}
