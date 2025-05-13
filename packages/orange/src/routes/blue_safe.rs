use core::time::Duration;

use evian::{
    math::IntoAngle,
    motion::{Basic, Seeking},
};
use vexide::time::sleep;

use crate::Robot;

// practice: good
impl Robot {
    pub async fn blue(&mut self) {
        self.drivetrain.tracking.set_heading(326.0.deg());
        self.intake.enable_jam_prevention();

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

        // Alliance stake
        self.intake.disable_jam_prevention();
        self.lady_brown.set_target(Self::LADY_BROWN_RAISED);
        sleep(Duration::from_millis(250)).await;
        self.intake.set_top_voltage(12.0);
        sleep(Duration::from_millis(1000)).await;

        self.intake.set_top_voltage(-3.0);
        sleep(Duration::from_millis(250)).await;

        self.lady_brown.set_target(Self::LADY_BROWN_FLAT);
        self.intake.set_top_voltage(0.0);
        sleep(Duration::from_millis(1000)).await;

        // Goal
        seeking.move_to_point(dt, (-15.5, 11.0)).reverse().await;
        self.lady_brown.set_target(Self::LADY_BROWN_LOWERED);
        self.intake.enable_jam_prevention();

        basic.turn_to_heading(dt, 180.0.deg()).await;
        basic
            .drive_distance_at_heading(dt, -26.0, 180.0.deg())
            .with_linear_output_limit(4.0)
            .await;
        _ = self.clamp.set_high();

        sleep(Duration::from_millis(500)).await;

        // First stack
        self.intake.set_voltage(12.0);
        seeking.move_to_point(dt, (-31.0, 9.5)).await;
        sleep(Duration::from_millis(350)).await;

        // Second stack
        _ = self.intake.raise();

        basic.turn_to_heading(dt, 110.0.deg()).await;
        seeking.move_to_point(dt, (-33.0, 27.0)).await;
        sleep(Duration::from_millis(500)).await;

        // Clear bottom of stack.
        basic.drive_distance(dt, -5.0).await;

        basic
            .turn_to_heading(dt, 145.0.deg())
            .without_tolerance_duration()
            .await;
        basic.drive_distance_at_heading(dt, 15.0, 145.0.deg()).await;

        basic
            .turn_to_heading(dt, 93.0.deg())
            .without_tolerance_duration()
            .await;

        // Stack at line
        seeking.move_to_point(dt, (-45.0, 42.0)).await;
        sleep(Duration::from_millis(1000)).await;
        _ = self.intake.lower();
        sleep(Duration::from_millis(250)).await;

        // Final
        basic
            .drive_distance_at_heading(dt, -40.0, 135.0.deg())
            .await;
        // seeking
        //     .move_to_point(dt, (-32.0, 14.0))
        //     .reverse()
        //     .await;
        basic.turn_to_heading(dt, 225.0.deg()).await;
        self.lady_brown.set_target(Self::LADY_BROWN_FLAT);
        seeking
            .move_to_point(dt, (-46.0, -12.0))
            .with_linear_output_limit(4.0)
            .with_timeout(Duration::from_secs_f64(2.5))
            .await;
        self.intake.set_voltage(12.0);
        sleep(Duration::from_millis(800)).await;

        basic.drive_distance(dt, -14.0).await;
        _ = self.intake.raise();
        basic.drive_distance(dt, 12.0).await;
        sleep(Duration::from_millis(800)).await;
        self.intake.set_bottom_voltage(0.0);
        self.lady_brown.set_target(Self::LADY_BROWN_LOWERED);

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
        _ = self.clamp.set_low();
        basic.drive_distance_at_heading(dt, -13.0, 45.0.deg())
            .with_timeout(Duration::from_millis(800))
            .await;
        self.intake.set_voltage(0.0);
        _ = self.right_arm.set_low();

        // Touch
        basic.drive_distance_at_heading(dt, 50.0, 45.0.deg()).await;
        self.lady_brown.set_target(Self::LADY_BROWN_FLAT);
    }
}
