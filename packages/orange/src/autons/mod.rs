use alloc::boxed::Box;
use core::{error::Error, f64::{consts::PI, MAX}, time::Duration};

use aubie2::subsystems::{intake::RejectColor, lady_brown::LadyBrownTarget};
use evian::{
    control::{AngularPid, Pid, Tolerances},
    differential::motion::{BasicMotion, Seeking},
    prelude::*,
};
use vexide::prelude::*;

use crate::{Robot, LADY_BROWN_LOWERED, LADY_BROWN_RAISED, LADY_BROWN_SCORED};

mod red_positive_right;
mod blue_positive_right;
mod red_carter_special;
mod skills;
mod testing;

pub use red_positive_right::red_positive_right;
pub use blue_positive_right::blue_positive_right;
pub use red_carter_special::red_carter_special;
pub use skills::skills;
pub use testing::testing;

const LINEAR_PID: Pid = Pid::new(1.15, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);

const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(5.0)
    .velocity_tolerance(0.4)
    .tolerance_duration(Duration::from_millis(25))
    .timeout(Duration::from_secs(10));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(8.0 * (PI / 180.0))
    .velocity_tolerance(0.09)
    .tolerance_duration(Duration::from_millis(15))
    .timeout(Duration::from_secs(10));
