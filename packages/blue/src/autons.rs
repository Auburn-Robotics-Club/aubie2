use alloc::boxed::Box;
use core::{error::Error, f64::consts::PI, time::Duration};

use aubie2::subsystems::{intake::RejectColor, lady_brown::LadyBrownTarget};
use evian::{
    control::{AngularPid, Pid, Tolerances}, differential::motion::{BasicMotion, Seeking}, drivetrain, prelude::*
};
use vexide::prelude::*;

use crate::{Robot, LADY_BROWN_LOWERED, LADY_BROWN_RAISED, LADY_BROWN_SCORED};

const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);

const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(5.0)
    .velocity_tolerance(0.5)
    .tolerance_duration(Duration::from_millis(15))
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

    seeking.move_to_point(dt, (24.0, 24.0)).await;
    seeking.move_to_point(dt, (0.0, 0.0)).await;

    Ok(())
}

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
    seeking.move_to_point(dt, (-22.0, 27.0)).await;
    _ = bot.clamp.set_high();

    // Intake first red ring.
    bot.intake.set_voltage(12.0);
    basic.turn_to_heading(dt, 96.0.deg()).await;
    basic.drive_distance(dt, 24.0).await;
    sleep(Duration::from_millis(500)).await;

    // Intake second red ring.
    basic.drive_distance(dt, -16.0).await;
    basic.turn_to_heading(dt, -19.0.deg()).await;
    basic.drive_distance(dt, 22.0).await;
    sleep(Duration::from_millis(700)).await;

    // Intake third/fourth ring.
    basic.turn_to_heading(dt, 224.0.deg()).await;

    seeking.move_to_point(dt, (-36.0, -12.0)).await;
    sleep(Duration::from_millis(1500)).await;

    // turn around and score goal
    basic.drive_distance(dt, -18.0).await;
    basic.turn_to_heading(dt, 38.0.deg()).await;
    basic.drive_distance(dt, -18.0).await;
    _ = bot.clamp.set_low();
    basic.drive_distance(dt, 16.0).await;

    // Rush for wallstake ring
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(LADY_BROWN_RAISED));
    basic.turn_to_heading(dt, 90.0.deg()).await;
    seeking.move_to_point(dt, (-44.0, 52.0)).await;
    sleep(Duration::from_millis(500)).await;
    bot.intake.set_voltage(0.0);

    // Align ring lol
    sleep(Duration::from_millis(500)).await;
    bot.intake.set_voltage(12.0);
    sleep(Duration::from_millis(250)).await;
    bot.intake.set_voltage(0.0);
    sleep(Duration::from_millis(250)).await;
    bot.intake.set_voltage(12.0);
    sleep(Duration::from_millis(150)).await;
    bot.intake.set_voltage(0.0);
    sleep(Duration::from_millis(250)).await;
    bot.intake.set_voltage(12.0);
    sleep(Duration::from_millis(150)).await;
    bot.intake.set_voltage(0.0);

    // Line up wallstake and score
    basic.drive_distance(dt, -5.5).await; // fuck this
    basic.turn_to_heading(dt, 180.0.deg()).await;
    basic.drive_distance(dt, 8.0).await;
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(LADY_BROWN_SCORED));
    sleep(Duration::from_secs(1)).await;
    bot.lady_brown
        .set_target(LadyBrownTarget::Position(LADY_BROWN_LOWERED));
    basic.drive_distance(dt, -18.0).await;

    // Clamp second goal
    basic.turn_to_heading(dt, 230.0.deg()).await;
    seeking.move_to_point(dt, (1.0, 78.0)).await;
    _ = bot.clamp.set_high();

    // Get first goal ring
    bot.intake.set_voltage(12.0);
    basic.turn_to_heading(dt, 185.0.deg()).await;
    basic.drive_distance(dt, 35.0).await;
    basic.drive_distance(dt, -2.0).await;
    sleep(Duration::from_millis(500)).await;

    basic.turn_to_heading(dt, 95.0.deg()).await;
    basic.drive_distance(dt, 30.0).await;
    basic.drive_distance(dt, -7.0).await;
    sleep(Duration::from_millis(500)).await;

    basic.turn_to_heading(dt, 5.0.deg()).await;
    basic.drive_distance(dt, 25.0).await;
    basic.drive_distance(dt, -10.0).await;
    sleep(Duration::from_millis(700)).await;

    // Intake final ring
    
    basic.drive_distance(dt, -16.0).await;
    basic.turn_to_heading(dt, 135.0.deg()).await;
    basic.drive_distance(dt, 24.0).await;

    // Score final goal
    basic.drive_distance(dt, -20.0).await;
    basic.turn_to_heading(dt, 315.0.deg()).await;
    basic.drive_distance(dt, -20.0).await;
    _ = bot.clamp.set_low();
    bot.intake.set_voltage(0.0);
    basic.drive_distance(dt, 8.0).await;
    basic.drive_distance(dt, -8.0).await;
    basic.drive_distance(dt, 20.0).await;

    // Blue alliance
    bot.intake.set_voltage(12.0);
    basic.turn_to_heading(dt, 30.0.deg()).await;
    basic.drive_distance(dt, 42.0).await;

    println!("{}", dt.tracking.position());


    Ok(())
}
