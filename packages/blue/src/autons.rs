use alloc::boxed::Box;
use core::{error::Error, time::Duration};

use evian::{
    control::{AngularPid, Pid, Tolerances},
    differential::motion::{BasicMotion, Seeking},
    prelude::*,
};
use vexide::prelude::*;

use crate::Robot;

const LINEAR_PID: Pid = Pid::new(0.5, 0.0, 0.0, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);

pub async fn tuning(bot: &mut Robot) -> Result<(), Box<dyn Error>> {
    let dt = &mut bot.drivetrain;
    let mut seeking = Seeking {
        distance_controller: LINEAR_PID,
        angle_controller: ANGULAR_PID,
        tolerances: Tolerances::new()
            .error_tolerance(0.3)
            .tolerance_duration(Duration::from_millis(100))
            .timeout(Duration::from_secs(2)),
    };
    let mut basic = BasicMotion {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: Tolerances::new()
            .error_tolerance(0.3)
            .tolerance_duration(Duration::from_millis(100))
            .timeout(Duration::from_secs(2)),
        angular_tolerances: Tolerances::new()
            .error_tolerance(0.3)
            .tolerance_duration(Duration::from_millis(100))
            .timeout(Duration::from_secs(2)),
    };

    basic.turn_to_heading(dt, 0.0.deg()).await;
    println!("Settled");

    seeking.move_to_point(dt, (0.0, 10.0)).await;

    Ok(())
}
