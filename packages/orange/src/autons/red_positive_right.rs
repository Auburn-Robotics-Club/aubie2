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

pub async fn red_positive_right(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
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

    // Goal rush
    _ = bot.grabber.extend();
    basic.linear_controller.set_kp(2.0);
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(0));
    basic.drive_distance_at_heading(dt, 37.0, 75.0.deg()).await;
    sleep(Duration::from_millis(75)).await;
    _ = bot.grabber.retract();
    _ = bot.grabber.pinch();
    sleep(Duration::from_millis(50)).await;
    basic.linear_controller.set_kp(1.0);
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(15));
    basic.drive_distance_at_heading(dt, -18.0, 105.0.deg()).await;
    _ = bot.grabber.extend();
    
    // Release goal
    _ = bot.grabber.release();
    sleep(Duration::from_millis(500)).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.7));
    basic.drive_distance_at_heading(dt, -16.0, 82.0.deg()).await;
    basic.linear_controller.set_output_limit(None);
    sleep(Duration::from_millis(500)).await;
    _ = bot.grabber.retract();
    sleep(Duration::from_millis(500)).await;

    basic.turn_to_heading(dt, 273.0.deg()).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance_at_heading(dt, -31.0, 265.0.deg()).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE));

    _ = bot.clamp.set_high();
    
    basic.turn_to_heading(dt, 328.0.deg()).await;
    bot.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.3));
    basic.drive_distance(dt, 30.0).await;
    sleep(Duration::from_millis(350)).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 1.0));
    basic.drive_distance(dt, -20.0).await;
    
    basic.turn_to_heading(dt, 277.0.deg()).await;
    basic.linear_tolerances.timeout = Some(Duration::from_secs(2));
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.5));
    basic.drive_distance(dt, 31.0).await;
    basic.turn_to_heading(dt, 322.0.deg()).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.2));
    basic.drive_distance(dt, 19.0).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.4));
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -16.0).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 1.0));

    basic.turn_to_heading(dt, 135.0.deg()).await;
    _ = bot.clamp.set_low();
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance_at_heading(dt, 66.0, 135.0.deg()).await;
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(Position::from_degrees(200.0)));

    Ok(())
}