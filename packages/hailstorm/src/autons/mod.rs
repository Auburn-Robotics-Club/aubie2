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

use crate::Robot;

mod red_left;
mod testing;

pub use red_left::red_left;
pub use testing::testing;

const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
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
