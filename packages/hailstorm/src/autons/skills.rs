use alloc::boxed::Box;
use vexide::prelude::Motor;
use core::error::Error;

use evian::differential::motion::BasicMotion;

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::Robot;

pub async fn skills(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    let mut basic = BasicMotion {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
    };

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance(dt, -200.0).await;
    basic.drive_distance(dt, 40.0).await;

    Ok(())
}
