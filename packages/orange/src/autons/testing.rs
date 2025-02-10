use alloc::boxed::Box;
use core::error::Error;

// use evian::differential::motion::{BasicMotion, Seeking};

// use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::Robot;

pub async fn testing(_bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    // let dt = &mut bot.drivetrain;
    // let mut seeking = Seeking {
    //     linear_controller: LINEAR_PID,
    //     angular_controller: ANGULAR_PID,
    //     tolerances: LINEAR_TOLERANCES,
    // };
    // let mut basic = BasicMotion {
    //     linear_controller: LINEAR_PID,
    //     angular_controller: ANGULAR_PID,
    //     linear_tolerances: LINEAR_TOLERANCES,
    //     angular_tolerances: ANGULAR_TOLERANCES,
    // };

    Ok(())
}
