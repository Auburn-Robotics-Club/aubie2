use alloc::boxed::Box;
use core::{
    error::Error,
    f64::{consts::PI, MAX},
    time::Duration,
};

use aubie2::subsystems::{intake::RejectColor, lady_brown::LadyBrownTarget};
use evian::{
    differential::motion::{BasicMotion, Seeking},
    prelude::*,
};
use vexide::prelude::*;

use super::{ANGULAR_PID, ANGULAR_TOLERANCES, LINEAR_PID, LINEAR_TOLERANCES};
use crate::{Robot, LADY_BROWN_LOWERED, LADY_BROWN_RAISED, LADY_BROWN_SCORED};

pub async fn blue_positive_right(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    bot.intake.set_reject_color(Some(RejectColor::Red));
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
    basic.drive_distance_at_heading(dt, 36.0, 144.0.deg()).await;
    _ = bot.grabber.pinch();
    basic.drive_distance(dt, -1.0).await;
    sleep(Duration::from_millis(15)).await;
    basic.linear_controller.set_kp(1.0);
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(15));

    // Drag goal back and release from grabber
    basic.angular_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.7));
    basic.drive_distance_at_heading(dt, -20.0, 128.0.deg()).await;
    basic.angular_controller.set_output_limit(None);
    _ = bot.grabber.release();
    sleep(Duration::from_millis(250)).await;
    basic.drive_distance(dt, -4.0).await;
    _ = bot.grabber.retract();
    sleep(Duration::from_millis(250)).await;

    // Clamp goal
    let goal_angle = 124.0.deg();
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.5));
    basic.drive_distance_at_heading(dt, -25.0, goal_angle).await;
    basic.linear_controller.set_output_limit(None);
    
    sleep(Duration::from_millis(500)).await;

    _ = bot.clamp.set_high();

    // First stack
    let stack_angle = 154.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;
    bot.intake.set_voltage(10.0);
    _ = bot.intake_raiser.set_high();

    basic.drive_distance_at_heading(dt, 42.0, stack_angle).await;
    _ = bot.intake_raiser.set_low();

    sleep(Duration::from_millis(250)).await;
    basic.drive_distance_at_heading(dt, -5.0, stack_angle).await;
    basic.drive_distance_at_heading(dt, 29.0, stack_angle).await;

    // Intake top of second stack
    let stack_angle = 90.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;
    _ = bot.intake_raiser.set_high();
    basic.drive_distance_at_heading(dt, 8.0, stack_angle).await;
    _ = bot.intake_raiser.set_low();
    sleep(Duration::from_millis(250)).await;
    basic.drive_distance_at_heading(dt, -5.0, stack_angle).await;
    sleep(Duration::from_millis(500)).await;

    seeking.move_to_point(dt, (-15.0, 15.0)).await;

    // Third stack
    let stack_angle = 225.0.deg();
    basic.turn_to_heading(dt, stack_angle).await;

    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.2));
    basic.linear_tolerances.timeout = Some(Duration::from_secs(4));
    basic.angular_tolerances.timeout = Some(Duration::from_secs(4));
    basic.drive_distance_at_heading(dt, 40.0, stack_angle).await;
    basic.linear_controller.set_output_limit(None);

    basic
        .linear_controller
        .set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.4));
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
    basic.drive_distance(dt, -18.0).await;

    seeking.move_to_point(dt, (-30.0, 24.0)).await;

    Ok(())
}
