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

pub async fn skills(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
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

    // Grab goal
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance_at_heading(dt, -34.0, 245.0.deg()).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE));
    _ = bot.clamp.set_high();

    sleep(Duration::from_millis(800)).await;

    // Intake first red ring.
    bot.intake.set_voltage(12.0);
    basic.turn_to_heading(dt, 84.0.deg()).await;
    basic.drive_distance_at_heading(dt, 24.0, 84.0.deg()).await;
    bot.intake.set_voltage(10.0);
    sleep(Duration::from_millis(500)).await;

    // Intake second red ring.
    basic.drive_distance(dt, -16.0).await;
    basic.turn_to_heading(dt, 198.0.deg()).await;
    basic.drive_distance(dt, 23.0).await;
    sleep(Duration::from_millis(700)).await;

    // Intake third/fourth ring.
    basic.turn_to_heading(dt, 313.0.deg()).await;

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.4));
    basic.drive_distance_at_heading(dt, 51.0, 313.0.deg()).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE));    
    sleep(Duration::from_millis(1500)).await;

    // turn around and score goal
    basic.drive_distance(dt, -18.0).await;
    basic.turn_to_heading(dt, 133.0.deg()).await;
    _ = bot.clamp.set_low();
    basic.drive_distance(dt, -14.0).await;
    basic.drive_distance(dt, 16.0).await;

    // Rush for wallstake ring
    bot.intake.set_voltage(11.0);
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(LADY_BROWN_RAISED));
    basic.turn_to_heading(dt, 90.0.deg()).await;
    seeking.move_to_point(dt, (40.0, 50.0)).await;
    sleep(Duration::from_millis(500)).await;
    bot.intake.set_voltage(0.0);

    // Align ring lol
    sleep(Duration::from_millis(500)).await;
    bot.intake.set_voltage(9.0);
    sleep(Duration::from_millis(150)).await;
    bot.intake.set_voltage(0.0);

    // Line up wallstake and score
    basic.drive_distance(dt, -4.0).await;
    basic.turn_to_heading(dt, 0.0.deg()).await;
    basic.drive_distance(dt, 7.0).await;
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(LADY_BROWN_SCORED));
    sleep(Duration::from_secs(1)).await;
    basic.drive_distance(dt, -18.0).await;
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(LADY_BROWN_LOWERED));

    // Clamp second goal
    basic.turn_to_heading(dt, 315.0.deg()).await;
    seeking.move_to_point(dt, (-5.0, 77.0)).await;
    _ = bot.clamp.set_high();

    // Get first goal ring
    bot.intake.set_voltage(12.0);
    basic.turn_to_heading(dt, 355.0.deg()).await;
    basic.drive_distance(dt, 35.0).await;
    basic.drive_distance(dt, -2.0).await;
    sleep(Duration::from_millis(500)).await;

    basic.turn_to_heading(dt, 87.0.deg()).await;
    basic.drive_distance(dt, 27.0).await;
    basic.drive_distance(dt, -4.0).await;
    sleep(Duration::from_millis(500)).await;

    basic.turn_to_heading(dt, 175.0.deg()).await;
    basic.drive_distance(dt, 25.0).await;
    basic.drive_distance(dt, -2.0).await;
    sleep(Duration::from_millis(700)).await;

    seeking.move_to_point(dt, (37.0, 112.0)).await;
    _ = bot.clamp.set_low();
    basic.drive_distance(dt, 5.0).await;
    basic.drive_distance(dt, -5.0).await;

    // seeking.move_to_point(dt, (0.4, 76.0)).await;

    // basic.turn_to_heading(dt, 315.0.deg()).await;
    seeking.move_to_point(dt, (-11.0, 112.0)).await;
    // _ = bot.clamp.set_high();

    Ok(())
}
