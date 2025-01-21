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
use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};

pub async fn red_carter_special(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    bot.intake.set_reject_color(Some(RejectColor::Blue));
    dt.tracking.set_heading(90.0.deg());
    
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


    // Rush
    _ = bot.grabber.extend();
    seeking.move_to_point(dt, (-22.0, 34.0)).await;
    // basic.drive_distance_at_heading(dt, 46.0, 145.0.deg()).await;
    basic.drive_distance(dt, 0.0);
    log::info!("{}", dt.tracking.position());

    Ok(())
}