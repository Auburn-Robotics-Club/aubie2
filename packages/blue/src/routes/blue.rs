use core::time::Duration;

use evian::{
    math::IntoAngle,
    motion::{Basic, Seeking},
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn blue(&mut self) {
        self.drivetrain.tracking.set_heading(60.0.deg());
        // self.intake.set_reject_color(Some(RingColor::Red));

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

        // Goal Rush
        _ = self.right_arm.set_high();
        seeking
            .move_to_point(dt, (9.5, 25.0))
            .with_linear_kp(2.0)
            .without_tolerance_duration()
            .await;
        _ = self.pinchers.set_high();

        // Drag back, unpinch.
        basic
            .drive_distance_at_heading(dt, -26.0, 100.0.deg())
            .with_timeout(Duration::from_secs_f64(1.5))
            .await;
        _ = self.pinchers.set_low();
        seeking
            .move_to_point(dt, (2.0, 0.0))
            .reverse()
            .with_timeout(Duration::from_millis(500))
            .await;
        basic
            .drive_distance_at_heading(dt, -8.0, 80.0.deg())
            .with_timeout(Duration::from_millis(500))
            .await;
        _ = self.right_arm.set_low();
        sleep(Duration::from_millis(500)).await;

        // Clamp
        let goal_angle = 267.5.deg();
        basic.turn_to_heading(dt, goal_angle).await;
        basic
            .drive_distance_at_heading(dt, -30.0, goal_angle)
            .with_linear_output_limit(3.0)
            .await;
        _ = self.clamp.set_high();

        sleep(Duration::from_millis(250)).await;
        self.intake.set_top_voltage(12.0);
        sleep(Duration::from_millis(500)).await;

        // Top of stack
        self.intake.set_voltage(12.0);
        _ = self.intake.raise();

        basic.turn_to_heading(dt, 340.0.deg()).await;
        seeking.move_to_point(dt, (17.0, 16.0)).await;

        _ = self.intake.lower();
        basic.drive_distance(dt, -8.0).await;

        // Final Path
        basic.turn_to_heading(dt, 225.0.deg()).await;
        seeking.move_to_point(dt, (0.0, 7.0)).await;

        basic.turn_to_heading(dt, 315.0.deg()).await;

        seeking
            .move_to_point(dt, (16.0, -7.5))
            .with_linear_output_limit(3.0)
            .await;
        
        // Clear the ring
        sleep(Duration::from_millis(400)).await;
        self.intake.set_bottom_voltage(-12.0);
        basic.turn_to_heading(dt, 235.0.deg())
            .without_tolerance_duration().await;
        _ = self.left_arm.set_high();
        sleep(Duration::from_millis(250)).await;

        basic.turn_to_heading(dt, 330.0.deg())
            .without_tolerance_duration()
            .await;

        basic.turn_to_heading(dt, 315.0.deg()).await;

        _ = self.left_arm.set_low();
        self.intake.set_voltage(0.0);

        // Corner
        seeking
            .move_to_point(dt, (36.0, -27.0))
            .with_linear_output_limit(4.0)
            .with_timeout(Duration::from_secs(3))
            .await;

        self.intake.set_voltage(12.0);
        sleep(Duration::from_millis(1000)).await;

        basic.drive_distance(dt, -14.0).await;
        basic.drive_distance(dt, 13.0)
            .with_linear_output_limit(6.0).await;
        basic.drive_distance(dt, -15.0).await;

        // Alliance stake
        basic.turn_to_heading(dt, 180.0.deg()).await;

        futures::join!(
            async {
                sleep(Duration::from_millis(800)).await;
                self.lady_brown.set_target(Self::LADY_BROWN_RAISED);
            },
            async {
                seeking
                    .move_to_point(dt, (-30.0, -22.0))
                    .with_linear_output_limit(6.0)
                    .await;
            }
        );

        basic.turn_to_heading(dt, 270.0.deg()).await;
        sleep(Duration::from_millis(500)).await;
        self.intake.set_top_voltage(-1.0);

        seeking
            .move_to_point(dt, (-30.0, -19.5))
            .reverse()
            .with_tolerance_duration(Duration::from_millis(50))
            .await;

        self.lady_brown.set_target(Self::LADY_BROWN_FLAT);
        sleep(Duration::from_secs(1)).await;

        // Go touch
        basic.drive_distance(dt, -10.0).await;
        self.lady_brown.set_target(Self::LADY_BROWN_UP);

        basic.turn_to_heading(dt, 90.0.deg()).await;
        basic.drive_distance(dt, 12.0).await;

        self.lady_brown.set_target(Self::LADY_BROWN_FLAT);
    }
}
