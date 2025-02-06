use alloc::boxed::Box;
use vexide::prelude::sleep;
use core::{error::Error, time::Duration};

use evian::{differential::motion::BasicMotion, math::IntoAngle, prelude::TracksHeading};

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::Robot;

pub async fn testing(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;

    let mut basic = BasicMotion {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
    };

    dt.tracking.set_heading(90.0.deg());

    log::debug!("{}", dt.tracking.heading().as_degrees());

    basic.turn_to_heading(dt, 45.0.deg()).await;
    dt.tracking.set_heading(90.0.deg());

    sleep(Duration::from_millis(500)).await;

    log::debug!("{}", dt.tracking.heading().as_degrees());

    Ok(())
}
