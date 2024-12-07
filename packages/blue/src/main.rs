#![no_main]
#![no_std]

extern crate alloc;

mod autons;

use core::time::Duration;

use aubie2::{
    subsystems::{
        lady_brown::{LadyBrown, LadyBrownTarget},
        Intake,
    },
    theme::THEME_WAR_EAGLE,
};
use evian::{control::Pid, prelude::*};
use vexide::prelude::*;

const LADY_BROWN_LOWERED: Position = Position::from_degrees(327.0);
const LADY_BROWN_RAISED: Position = Position::from_degrees(289.0);
const LADY_BROWN_SCORED: Position = Position::from_degrees(165.0);

pub struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, ParallelWheelTracking>,
    intake: Intake,
    lady_brown: LadyBrown,
    clamp: AdiDigitalOut,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        autons::skills(self).await.unwrap();
    }

    async fn driver(&mut self) {
        self.intake.set_reject_color(None);
        loop {
            let state = self.controller.state().unwrap_or_default();

            // Single-stick arcade joystick control
            _ = self.drivetrain.motors.set_voltages(
                DifferentialVoltages::from_arcade(
                    state.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                    state.left_stick.x() * Motor::V5_MAX_VOLTAGE,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );

            // Raise/slower ladybrown when B is pressed.
            if state.button_b.is_now_pressed() {
                self.lady_brown.set_target(match self.lady_brown.target() {
                    LadyBrownTarget::Position(state) => match state {
                        LADY_BROWN_LOWERED => LadyBrownTarget::Position(LADY_BROWN_RAISED),
                        LADY_BROWN_RAISED => LadyBrownTarget::Position(LADY_BROWN_LOWERED),
                        _ => unreachable!(),
                    },
                    LadyBrownTarget::Manual(_) => LadyBrownTarget::Position(LADY_BROWN_RAISED),
                });
            }

            // Manual ladybrown control using R1/R2.
            if state.button_l1.is_pressed() {
                self.lady_brown
                    .set_target(LadyBrownTarget::Manual(MotorControl::Voltage(
                        Motor::V5_MAX_VOLTAGE,
                    )));
            } else if state.button_l2.is_pressed() {
                self.lady_brown
                    .set_target(LadyBrownTarget::Manual(MotorControl::Voltage(
                        -Motor::V5_MAX_VOLTAGE,
                    )));
            } else if let LadyBrownTarget::Manual(_) = self.lady_brown.target() {
                self.lady_brown
                    .set_target(LadyBrownTarget::Manual(MotorControl::Brake(
                        BrakeMode::Hold,
                    )));
            }

            // Intake control - R1/R2.
            if state.button_r1.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if state.button_r2.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.set_voltage(0.0);
            }

            // A to toggle mogo mech.
            if state.button_a.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            sleep(Motor::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    println!("Start");
    let mut imu = InertialSensor::new(peripherals.port_21);

    #[allow(clippy::collapsible_if)]
    if imu.calibrate().await.is_err() {
        if imu.calibrate().await.is_err() {
            sleep(Duration::from_secs(3)).await;
        }
    }
    println!("Calibrated");

    let robot = Robot {
        // Controller
        controller: peripherals.primary_controller,

        // Drivetrain Model & Localization
        drivetrain: {
            // Left/right motors shared between drivetrain and odometry.
            let left_motors = shared_motors![
                Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
            ];
            let right_motors = shared_motors![
                Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_16, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
            ];

            // Drivetrain Model
            Drivetrain::new(
                Differential::new(left_motors.clone(), right_motors.clone()),
                ParallelWheelTracking::new(
                    Vec2::new(0.0, 0.0),
                    270.0.deg(),
                    TrackingWheel::new(left_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                    TrackingWheel::new(right_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                    Some(imu),
                ),
            )
        },

        // Intake
        intake: Intake::new(
            [
                Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
            ],
            OpticalSensor::new(peripherals.port_5),
            None,
        ),

        // Lady Brown Arm
        lady_brown: LadyBrown::new(
            [
                Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
                Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward),
            ],
            RotationSensor::new(peripherals.port_4, Direction::Forward),
            Pid::new(0.45, 0.0, 0.001, None),
        ),

        // Mogo
        clamp: AdiDigitalOut::new(peripherals.adi_h),
    };

    robot.compete().await;
}
