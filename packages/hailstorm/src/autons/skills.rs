use alloc::boxed::Box;
use core::{error::Error, time::Duration};

use evian::{differential::motion::BasicMotion, math::IntoAngle};
use vexide::prelude::{sleep, Motor};

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::Robot;

pub async fn skills(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    dt.tracking.set_heading(270.0.deg());
    let mut basic = BasicMotion {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
    };

    sleep(Duration::from_millis(500)).await;

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic
        .drive_distance_at_heading(dt, -34.0, 245.0.deg())
        .await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE));
    _ = bot.clamp.set_high();

    sleep(Duration::from_millis(800)).await;

    Ok(())
}
