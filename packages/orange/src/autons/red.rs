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

pub async fn red(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    bot.intake.set_reject_color(Some(RejectColor::Blue));
    dt.tracking.set_heading(90.0.deg());

    let mut seeking = Seeking {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
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
    basic.drive_distance_at_heading(dt, 38.0, 75.0.deg()).await;
    sleep(Duration::from_millis(60)).await;
    _ = bot.grabber.pinch();
    sleep(Duration::from_millis(25)).await;
    basic.linear_controller.set_kp(1.0);
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(15));
    basic.drive_distance_at_heading(dt, -16.0, 75.0.deg()).await;

    _ = bot.grabber.release();

    // Grab goal
    basic.drive_distance(dt, -27.0).await;
    _ = bot.grabber.retract();
    basic.turn_to_heading(dt, 0.0.deg()).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance_at_heading(dt, -24.0, 0.0.deg()).await;
    _ = bot.clamp.set_high();
    sleep(Duration::from_millis(800)).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.7));

    // Intake first stack
    bot.intake.set_voltage(14.0);
    basic.drive_distance_at_heading(dt, 47.0, 0.0.deg()).await;
    sleep(Duration::from_millis(500)).await; // wait for ring to intake before 90 degree turn

    // Intake second stack
    let stack_angle = 90.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;

    _ = bot.intake_raiser.set_high();
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.3));
    basic.drive_distance_at_heading(dt, 24.0, stack_angle).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.7));
    _ = bot.intake_raiser.set_low();

    basic.drive_distance_at_heading(dt, -8.0, stack_angle).await;

    // Intake third stack
    basic.turn_to_heading(dt, 0.0.deg()).await;
    basic.linear_tolerances.timeout = Some(Duration::from_secs(3));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(3));
    basic.drive_distance_at_heading(dt, 13.0, 0.0.deg()).await;

    basic.turn_to_heading(dt, 90.0.deg()).await;
    basic.turn_to_heading(dt, 90.0.deg()).await;
    basic.drive_distance_at_heading(dt, 30.0, 90.0.deg()).await;

    _ = dt.motors.brake(BrakeMode::Hold);
    sleep(Duration::from_millis(800)).await;

    // Drive back
    basic.drive_distance_at_heading(dt, -35.0, 45.0.deg()).await;

    // Final push
    let stack_angle = 315.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance_at_heading(dt, 43.0, stack_angle).await;

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.linear_tolerances.timeout = Some(Duration::from_secs(1));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(1));
    basic.drive_distance(dt, -16.0).await;
    basic.drive_distance(dt, 16.0).await;
    basic.drive_distance(dt, -16.0).await;
    basic.drive_distance(dt, 16.0).await;
    basic.drive_distance(dt, -16.0).await;
    basic.drive_distance(dt, 16.0).await;
    basic.drive_distance(dt, -16.0).await;
    basic.drive_distance(dt, 16.0).await;
    basic.drive_distance(dt, -16.0).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 1.0));

    basic.turn_to_heading(dt, 135.0.deg()).await;
    basic.turn_to_heading(dt, 135.0.deg()).await;

    _ = bot.clamp.set_low();
    basic.drive_distance_at_heading(dt, -18.0, 135.0.deg()).await;

    basic.drive_distance_at_heading(dt, 36.0, 90.0.deg()).await;
    basic.turn_to_heading(dt, 270.0.deg()).await;

    Ok(())
}
