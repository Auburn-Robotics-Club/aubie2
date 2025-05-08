use core::time::Duration;

use evian::{
    math::IntoAngle,
    motion::{Basic, Seeking},
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn blue_rush(&mut self) {
        self.drivetrain.tracking.set_heading(102.0.deg());

        let dt = &mut self.drivetrain;
        let mut basic = Basic {
            linear_controller: Robot::LINEAR_PID,
            angular_controller: Robot::ANGUALR_PID,
            linear_tolerances: Robot::LINEAR_TOLERANCES,
            angular_tolerances: Robot::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };
        let mut seeking = Seeking {
            linear_controller: Robot::LINEAR_PID,
            angular_controller: Robot::ANGUALR_PID,
            tolerances: Robot::LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };

        // Goal rush
        _ = self.left_arm.set_high();
        seeking
            .move_to_point(dt, (-11.0, 35.5))
            .with_linear_kp(2.0)
            .without_tolerance_duration()
            .await;
        _ = self.pinchers.set_high();

        basic
            .drive_distance_at_heading(dt, -17.0, 102.0.deg())
            .with_timeout(Duration::from_secs_f64(1.5))
            .await;

        _ = self.pinchers.set_low();
        basic.drive_distance(dt, 4.0).await;

        seeking.move_to_point(dt, (0.0, 0.0)).reverse().await;

        _ = self.left_arm.set_low();

        basic.turn_to_heading(dt, 180.0.deg()).await;

        basic
            .drive_distance_at_heading(dt, -26.0, 181.5.deg())
            .with_linear_output_limit(4.0)
            .await;
        _ = self.clamp.set_high();

        sleep(Duration::from_millis(500)).await;

        // First stack
        self.intake.set_voltage(12.0);
        seeking.move_to_point(dt, (-21.0, 2.0)).await;
        sleep(Duration::from_millis(350)).await;

        // Second stack
        _ = self.intake.raise();

        basic.turn_to_heading(dt, 90.0.deg()).await;

        seeking.move_to_point(dt, (-20.0, 18.0)).await;
        sleep(Duration::from_millis(500)).await;

        // Stack at line
        basic.drive_distance_at_heading(dt, -9.0, 135.0.deg()).await;
        basic.drive_distance_at_heading(dt, 26.5, 135.0.deg()).await;

        basic.turn_to_heading(dt, 96.0.deg()).await;
        seeking.move_to_point(dt, (-36.0, 36.0)).await;

        sleep(Duration::from_millis(1000)).await;

        // Final
        basic
            .drive_distance_at_heading(dt, -38.0, 136.0.deg())
            .await;
        _ = self.intake.lower();
        basic.turn_to_heading(dt, 225.0.deg()).await;

        seeking
            .move_to_point(dt, (-36.0, -15.0))
            .with_linear_output_limit(4.0)
            .with_timeout(Duration::from_secs_f64(2.5))
            .await;
        self.intake.set_voltage(12.0);
        sleep(Duration::from_millis(800)).await;

        basic.drive_distance(dt, -14.0).await;
        _ = self.intake.raise();
        basic
            .drive_distance(dt, 13.0)
            .with_linear_output_limit(4.0)
            .await;

        // Clear corner
        _ = self.intake.lower();
        self.intake.set_voltage(-1.0);
        basic.drive_distance(dt, -8.0).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;
        _ = self.right_arm.set_high();
        sleep(Duration::from_millis(800)).await;
        basic
            .drive_distance_at_heading(dt, 5.0, 145.0.deg())
            .without_tolerance_duration()
            .await;

        // Drop goal
        self.intake.set_voltage(-1.5);
        basic.turn_to_heading(dt, 45.0.deg()).await;
        basic.drive_distance_at_heading(dt, -16.0, 45.0.deg()).await;
        self.intake.set_voltage(0.0);
        _ = self.clamp.set_low();
        _ = self.right_arm.set_low();
        basic
            .drive_distance_at_heading(dt, -4.0, 45.0.deg())
            .with_linear_output_limit(4.0)
            .await;

        // Align to goal;
        basic.drive_distance_at_heading(dt, 32.0, 90.0.deg()).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;

        // // Touch
        // basic.drive_distance_at_heading(dt, 52.0, 135.0.deg()).await;
        // self.lady_brown.set_target(Self::LADY_BROWN_FLAT);
    }
}
