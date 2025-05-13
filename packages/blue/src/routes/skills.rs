use core::time::Duration;

use evian::{
    math::IntoAngle,
    motion::{Basic, Seeking},
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn skills(&mut self) {
        self.drivetrain.tracking.set_heading(270.0.deg());

        let dt = &mut self.drivetrain;
        let mut basic = Basic {
            linear_controller: Robot::LINEAR_PID,
            angular_controller: Robot::ANGUALR_PID,
            linear_tolerances: Robot::LINEAR_TOLERANCES,
            angular_tolerances: Robot::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(15)),
        };

        basic.drive_distance_at_heading(dt, -300.0, 270.0.deg())
            .with_linear_output_limit(0.35).await;
    }
}
