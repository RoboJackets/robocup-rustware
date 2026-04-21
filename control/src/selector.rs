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
    let team = if p0.is_set() {
        Team::Blue
    } else {
        Team::Yellow
    };
    let id = (!p1.is_set() as u8) << 2 | (!p2.is_set() as u8) << 1 | (!p3.is_set() as u8);
    (team, id)
}
