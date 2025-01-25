use alloc::boxed::Box;
use core::{error::Error, f64::consts::PI, time::Duration};

use evian::{
    differential::motion::{BasicMotion, Seeking},
    prelude::*,
};
use vexide::prelude::*;

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::Robot;

pub async fn red(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    dt.tracking.set_heading(270.0.deg());

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

    for motor in bot.intake.iter_mut() {
        _ = motor.set_voltage(Motor::V5_MAX_VOLTAGE);
    }

    basic.linear_controller.set_kp(1.5);
    basic.linear_tolerances.velocity_tolerance = None;
    basic.angular_tolerances.velocity_tolerance = Some(1.0);
    basic.angular_tolerances.error_tolerance = Some(16.0 * (PI / 180.0));
    basic.angular_tolerances.tolerance_duration = None;
    basic.linear_tolerances.tolerance_duration = None;
    basic
        .drive_distance_at_heading(dt, -30.0, 300.0.deg())
        .await;
    basic.linear_controller.set_kp(1.0);
    basic.linear_tolerances.velocity_tolerance = Some(0.4);
    basic.angular_tolerances.velocity_tolerance = Some(0.09);
    basic.angular_tolerances.error_tolerance = Some(8.0 * (PI / 180.0));
    basic.angular_tolerances.tolerance_duration = Some(Duration::from_millis(15));
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(15));
    _ = bot.doinker.set_high();

    sleep(Duration::from_millis(30)).await;
    basic.drive_distance(dt, 24.0).await;
    _ = bot.doinker.set_low();

    // Clamp goal
    basic.turn_to_heading(dt, 318.0.deg()).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance(dt, -14.0).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE));

    sleep(Duration::from_millis(250)).await;
    _ = bot.clamp.set_high();
    sleep(Duration::from_millis(250)).await;

    // Score first ring
    bot.lift.score().await;

    // Intake stack
    let stack_angle = 248.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;
    _ = bot.clamp.set_low();
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance_at_heading(dt, 24.0, stack_angle).await;
    println!("{}", dt.tracking.position());
    sleep(Duration::from_millis(1250)).await;

    // Get second goal
    let second_goal_angle = 182.0.deg();
    basic.turn_to_heading(dt, second_goal_angle).await;

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.4));
    basic.drive_distance_at_heading(dt, -44.0, second_goal_angle).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE));

    _ = bot.clamp.set_high();
    sleep(Duration::from_millis(250)).await;

    bot.lift.score_safe().await;
    sleep(Duration::from_millis(500)).await;
    _ = bot.clamp.set_low();
    basic.drive_distance_at_heading(dt, -14.0, 180.0.deg()).await;

    // Score on alliance stake
    let stake_sideways_dist = 14.25;
    let stake_forward_dist = 18.0;
    let stake_intake_dist = -8.0;
    let stake_reverse_dist = -7.5;

    basic.drive_distance_at_heading(dt, stake_sideways_dist, 180.0.deg()).await;

    basic.angular_tolerances.error_tolerance = Some(f64::to_radians(2.0));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(3));
    basic.angular_controller.set_output_limit(Some(8.0));
    basic.turn_to_heading(dt, 270.0.deg()).await;
    basic.angular_tolerances.error_tolerance = Some(f64::to_radians(8.0));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(10));
    basic.angular_controller.set_output_limit(None);

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.15));
    basic.drive_distance_at_heading(dt, stake_forward_dist, 270.0.deg()).await;
    basic
    .linear_controller
    .set_output_limit(Some(Motor::V5_MAX_VOLTAGE));
    basic.drive_distance_at_heading(dt, stake_intake_dist, 270.0.deg()).await;

    basic.angular_tolerances.error_tolerance = Some(f64::to_radians(2.0));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(3));
    basic.angular_controller.set_output_limit(Some(7.0));
    basic.turn_to_heading(dt, 90.0.deg()).await;
    basic.angular_tolerances.error_tolerance = Some(f64::to_radians(8.0));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(10));
    basic.angular_controller.set_output_limit(None);

    sleep(Duration::from_millis(250)).await;
    basic.drive_distance_at_heading(dt, stake_reverse_dist, 90.0.deg()).await;
    sleep(Duration::from_millis(250)).await;

    bot.lift.score_safe().await;
    sleep(Duration::from_millis(500)).await;

    for motor in bot.intake.iter_mut() {
        _ = motor.set_voltage(0.0);
    }

    basic
    .linear_controller
    .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.5));
    basic.drive_distance(dt, 34.0).await;

    Ok(())
}
