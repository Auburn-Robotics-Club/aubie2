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

const LINEAR_PID: Pid = Pid::new(1.15, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);

const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(5.0)
    .velocity_tolerance(0.4)
    .tolerance_duration(Duration::from_millis(25))
    .timeout(Duration::from_secs(10));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(8.0 * (PI / 180.0))
    .velocity_tolerance(0.09)
    .tolerance_duration(Duration::from_millis(15))
    .timeout(Duration::from_secs(10));

pub async fn _testing(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
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

    basic.drive_distance(dt, 24.0).await;
    basic.drive_distance(dt, -24.0).await;
    basic.drive_distance(dt, 24.0).await;

    Ok(())
}

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
    basic.drive_distance_at_heading(dt, 38.0, 75.0.deg()).await;
    sleep(Duration::from_millis(50)).await;
    _ = bot.grabber.pinch();
    sleep(Duration::from_millis(50)).await;
    basic.linear_controller.set_kp(1.0);
    basic.linear_tolerances.tolerance_duration = Some(Duration::from_millis(15));
    basic.drive_distance_at_heading(dt, -18.0, 105.0.deg()).await;
    
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
    basic.drive_distance(dt, -31.0).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE));

    _ = bot.clamp.set_high();
    
    basic.turn_to_heading(dt, 312.0.deg()).await;
    bot.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.3));
    basic.drive_distance(dt, 30.0).await;
    sleep(Duration::from_millis(350)).await;
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 1.0));
    basic.drive_distance(dt, -20.0).await;
    
    basic.turn_to_heading(dt, 277.0.deg()).await;
    basic.linear_tolerances.timeout = Some(Duration::from_secs(2));
    basic.linear_controller.set_output_limit(Some(Motor::V5_MAX_VOLTAGE * 0.5));
    basic.drive_distance(dt, 33.0).await;
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
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(Position::from_degrees(200.0)));
    basic.drive_distance_at_heading(dt, 58.0, 135.0.deg()).await;




    // seeking.move_to_point(dt, (-20.0, 22.0)).await;
    // seeking.move_to_point(dt, (8.0, 34.0)).await;
    // basic.drive_distance(dt, 3.0).await;
    // _ = bot.clamp.set_high();

    // bot.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
    // basic.drive_distance(dt, 24.0).await;


    Ok(())
}

#[allow(unused)]
pub async fn skills(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
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

    bot.intake.set_reject_color(Some(RejectColor::Blue));

    // Grab goal
    seeking.move_to_point(dt, (16.0, 26.0)).await;
    _ = bot.clamp.set_high();

    // Intake first red ring.
    bot.intake.set_voltage(12.0);
    basic.turn_to_heading(dt, 80.0.deg()).await;
    basic.drive_distance(dt, 24.0).await;
    bot.intake.set_voltage(10.0);
    sleep(Duration::from_millis(500)).await;

    // Intake second red ring.
    basic.drive_distance(dt, -16.0).await;
    basic.turn_to_heading(dt, 196.0.deg()).await;
    basic.drive_distance(dt, 22.0).await;
    sleep(Duration::from_millis(700)).await;

    // Intake third/fourth ring.
    basic.turn_to_heading(dt, 313.0.deg()).await;

    seeking.move_to_point(dt, (36.0, -12.0)).await;
    sleep(Duration::from_millis(1500)).await;

    // turn around and score goal
    basic.drive_distance(dt, -18.0).await;
    basic.turn_to_heading(dt, 133.0.deg()).await;
    basic.drive_distance(dt, -16.0).await;
    _ = bot.clamp.set_low();
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

    basic.turn_to_heading(dt, 84.0.deg()).await;
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


