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

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::{Robot, LADY_BROWN_LOWERED, LADY_BROWN_RAISED, LADY_BROWN_SCORED};

pub async fn testing(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    let mut seeking = Seeking {
        distance_controller: LINEAR_PID,
        angle_controller: ANGULAR_PID,
        tolerances: LINEAR_TOLERANCES,
    };
    let mut basic = BasicMotion {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
    };

    basic.drive_distance(dt, 24.0).await;
    basic.drive_distance(dt, -24.0).await;
    basic.drive_distance(dt, 24.0).await;

    Ok(())
}
