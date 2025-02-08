use alloc::boxed::Box;
use core::{
    error::Error,
    f64::{consts::PI, MAX},
    time::Duration,
};

use aubie2::subsystems::{intake::RejectColor, lady_brown::LadyBrownTarget};
use evian::{
    control::{AngularPid, Pid, Tolerances},
    differential::motion::{BasicMotion, Seeking},
    prelude::*,
};
use vexide::prelude::*;

use crate::{Robot, LADY_BROWN_LOWERED, LADY_BROWN_RAISED, LADY_BROWN_SCORED};

mod blue_carter_special;
mod blue;
mod red;
mod skills;
mod testing;

pub use blue_carter_special::blue_carter_special;
pub use blue::blue;
pub use red::red;
pub use skills::skills;
pub use testing::testing;

const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);

const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(4.0)
    .velocity_tolerance(0.25)
    .tolerance_duration(Duration::from_millis(15))
    .timeout(Duration::from_secs(10));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(8.0 * (PI / 180.0))
    .velocity_tolerance(0.09)
    .tolerance_duration(Duration::from_millis(15))
    .timeout(Duration::from_secs(10));
