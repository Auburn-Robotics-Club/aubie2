use alloc::boxed::Box;
use core::error::Error;

use evian::differential::motion::BasicMotion;

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::Robot;

pub async fn testing(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    // let seeking = Seeking {
    //     distance_controller: LINEAR_PID,
    //     angle_controller: ANGULAR_PID,
    //     tolerances: LINEAR_TOLERANCES,
    // };
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
