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

    _ = bot.lift.score().await;

    // Second stack
    let stack_angle = 240.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.5));
    basic.drive_distance_at_heading(dt, 25.0, stack_angle).await;

    sleep(Duration::from_millis(1000)).await;

    _ = bot.lift.score().await;

    // Third stack
    let stack_angle = 338.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;
    basic.drive_distance_at_heading(dt, 40.0, stack_angle).await;

    sleep(Duration::from_millis(1000)).await;

    _ = bot.lift.score().await;

    // Bar touch
    for motor in bot.intake.iter_mut() {
        _ = motor.set_voltage(0.0);
    }

    basic.turn_to_heading(dt, 70.0.deg()).await;
    basic.drive_distance_at_heading(dt, 12.0, 70.0.deg()).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.35));
    basic.drive_distance_at_heading(dt, 20.0, 90.0.deg()).await;

    Ok(())
}
