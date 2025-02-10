use alloc::boxed::Box;
use core::{error::Error, time::Duration};

use aubie2::subsystems::intake::RingColor;
use evian::{differential::motion::BasicMotion, prelude::*};
use vexide::prelude::*;

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::Robot;

pub async fn blue(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    bot.intake.set_reject_color(Some(RingColor::Red));
    dt.tracking.set_heading(90.0.deg());

    let mut basic = BasicMotion {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
    };

    // Intake deploy
    bot.intake.set_bottom_voltage(-Motor::V5_MAX_VOLTAGE);

    // Goal rush
    _ = bot.grabber.extend();
    basic.linear_controller.set_kp(2.0);
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(0));
    basic.drive_distance_at_heading(dt, 36.0, 142.0.deg()).await;
    _ = bot.grabber.pinch();
    basic.drive_distance(dt, -1.0).await;
    sleep(Duration::from_millis(15)).await;
    basic.linear_controller.set_kp(1.0);
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(15));

    // Drag goal back and release from grabber
    basic
        .angular_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.7));
    basic
        .drive_distance_at_heading(dt, -20.0, 128.0.deg())
        .await;
    basic.angular_controller.set_output_limit(None);
    _ = bot.grabber.release();
    sleep(Duration::from_millis(250)).await;
    basic.drive_distance(dt, -4.0).await;
    _ = bot.grabber.retract();
    sleep(Duration::from_millis(250)).await;

    // Clamp goal
    let goal_angle = 127.0.deg();
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.5));
    basic.drive_distance_at_heading(dt, -25.0, goal_angle).await;
    basic.linear_controller.set_output_limit(None);

    sleep(Duration::from_millis(500)).await;

    _ = bot.clamp.set_high();

    // First stack
    let stack_angle = 150.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;
    bot.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
    _ = bot.intake_raiser.set_high();

    basic.drive_distance_at_heading(dt, 42.0, stack_angle).await;
    _ = bot.intake_raiser.set_low();

    sleep(Duration::from_millis(250)).await;
    basic.drive_distance_at_heading(dt, -5.0, stack_angle).await;
    basic.drive_distance_at_heading(dt, 30.0, stack_angle).await;

    // Intake top of second stack
    let stack_angle = 90.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;
    _ = bot.intake_raiser.set_high();
    basic.drive_distance_at_heading(dt, 9.0, stack_angle).await;
    _ = bot.intake_raiser.set_low();
    sleep(Duration::from_millis(250)).await;
    basic.drive_distance_at_heading(dt, -6.0, stack_angle).await;
    sleep(Duration::from_millis(500)).await;

    // Third stack
    basic.turn_to_heading(dt, 135.0.deg()).await;
    basic.drive_distance(dt, -36.0).await;

    let stack_angle = 225.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.2));
    basic.linear_tolerances.timeout = Some(Duration::from_secs(4));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(4));
    basic.drive_distance_at_heading(dt, 39.0, stack_angle).await;
    basic.linear_controller.set_output_limit(None);

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.3));
    basic.linear_tolerances.timeout = Some(Duration::from_secs(2));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(2));
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -20.0).await;
    basic.drive_distance(dt, 20.0).await;
    basic.drive_distance(dt, -16.0).await;
    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 1.0));

    basic.turn_to_heading(dt, 45.0.deg()).await;
    _ = bot.clamp.set_low();
    basic.drive_distance_at_heading(dt, -18.0, 45.0.deg()).await;

    basic.drive_distance_at_heading(dt, 36.0, 90.0.deg()).await;
    basic.turn_to_heading(dt, 270.0.deg()).await;

    Ok(())
}
