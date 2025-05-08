#![no_main]
#![no_std]

extern crate alloc;

use core::time::Duration;

use aubie2::{
    hardware::{calibrate_imu, CustomEncoder},
    logger::SerialLogger,
    subsystems::{
        lady_brown::{LadyBrown, LadyBrownTarget},
        Intake,
    },
    theme::THEME_WAR_EAGLE,
};
use evian::{
    control::loops::{AngularPid, Pid},
    prelude::*,
};
use log::{info, LevelFilter};
use vexide::{prelude::*, time::Instant};

pub mod routes;

// MARK: Robot

pub struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, WheeledTracking>,
    intake: Intake,
    lady_brown: LadyBrown,
    clamp: AdiDigitalOut,
    left_arm: AdiDigitalOut,
    right_arm: AdiDigitalOut,
    pinchers: AdiDigitalOut,
}

impl Robot {
    // Measurements
    pub const TRACK_WIDTH: f64 = 11.5;
    pub const WHEEL_DIAMETER: f64 = 2.75;
    pub const TRACKING_WHEEL_DIAMETER: f64 = 2.0;

    pub const SIDEWAYS_TRACKING_WHEEL_OFFSET: f64 = -2.0;

    // Lady Brown Positions
    pub const LADY_BROWN_LOWERED: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(295.0));
    pub const LADY_BROWN_RAISED: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(269.0));
    pub const LADY_BROWN_UP: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(170.0));
    pub const LADY_BROWN_SCORED: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(140.0));
    pub const LADY_BROWN_FLAT: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(90.0));

    // Control Loops
    pub const LINEAR_PID: Pid = Pid::new(1.5, 0.1, 0.125, Some(3.0));
    pub const ANGUALR_PID: AngularPid =
        AngularPid::new(25.0, 2.0, 1.0, Some(Angle::from_degrees(5.0)));
    pub const LADY_BROWN_PID: Pid = Pid::new(0.19, 0.0, 0.01, None);

    // Tolerances
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(5.0)
        .velocity(0.25)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));
}

// MARK: Competition

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let start = Instant::now();

        self.red().await;

        info!("Route completed successfully in {:?}.", start.elapsed());
        info!(
            "Position: {}\nHeading: {}° ({}rad)",
            self.drivetrain.tracking.position(),
            self.drivetrain.tracking.heading().as_degrees(),
            self.drivetrain.tracking.heading().as_radians()
        );
    }

    async fn driver(&mut self) {
        if self.intake.is_raised().unwrap_or(true) {
            _ = self.intake.lower();
        }

        loop {
            let state = self.controller.state().unwrap_or_default();

            // Single-stick arcade joystick control.
            _ = self.drivetrain.motors.set_voltages(
                Voltages::from_arcade(
                    state.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                    state.left_stick.x() * Motor::V5_MAX_VOLTAGE,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );

            // Manual lady brown control with right joystick.
            if state.right_stick.y().abs() > 0.1 {
                self.lady_brown
                    .set_target(LadyBrownTarget::Manual(MotorControl::Voltage(
                        state.right_stick.y() * Motor::V5_MAX_VOLTAGE,
                    )));
            } else if let LadyBrownTarget::Manual(_) = self.lady_brown.target() {
                self.lady_brown
                    .set_target(LadyBrownTarget::Manual(MotorControl::Brake(
                        BrakeMode::Hold,
                    )));
            }

            // Lady Brown
            //
            // R2: Toggle raise/lower.
            // R2: Toggle score/raise
            let lady_brown_target = self.lady_brown.target();
            if state.button_r2.is_now_pressed() {
                self.lady_brown.set_target(Self::LADY_BROWN_LOWERED);
            } else if state.button_r1.is_now_pressed() {
                self.lady_brown.set_target(match lady_brown_target {
                    Self::LADY_BROWN_RAISED => Self::LADY_BROWN_SCORED,
                    _ => Self::LADY_BROWN_RAISED,
                });
            }

            // Intake
            //
            // B: Forwards
            // Down: Backwards
            if state.button_b.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if state.button_down.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.set_voltage(0.0);
            }

            // Left Arm
            //
            // Right: Toggle Extender
            if state.button_right.is_now_pressed() {
                _ = self.left_arm.toggle();
            }

            // Right Arm
            //
            // A: Toggle Extender
            if state.button_y.is_now_pressed() {
                _ = self.right_arm.toggle();
            }

            // Goal Rush Arm Pinchers
            //
            // L1: Toggle Pinchers
            if state.button_l1.is_now_pressed() {
                _ = self.pinchers.toggle();
            }

            // Clamp
            //
            // A: Toggle
            if state.button_a.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            // Hero's Journey
            if state.button_x.is_now_pressed() {
                _ = self.left_arm.toggle();
                _ = self.right_arm.toggle();
            }

            sleep(Motor::UPDATE_INTERVAL).await;
        }
    }
}

// MARK: Main

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    SerialLogger.init(LevelFilter::Trace).unwrap();

    // Solenoids: port_5

    let mut controller = peripherals.primary_controller;
    let mut display = peripherals.display;
    let mut imu = InertialSensor::new(peripherals.port_4);
    let enc = CustomEncoder::<8192>::new(peripherals.adi_g, peripherals.adi_h, Direction::Forward);

    calibrate_imu(&mut controller, &mut display, &mut imu).await;

    let robot = Robot {
        // Controller
        controller,

        // Drivetrain & Localization
        drivetrain: {
            // Left/right motors shared between drivetrain and odometry.
            let left_motors = shared_motors![
                Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
            ];
            let right_motors = shared_motors![
                Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
            ];

            Drivetrain::new(
                Differential::from_shared(left_motors.clone(), right_motors.clone()),
                WheeledTracking::new(
                    Vec2::default(),
                    90.0.deg(),
                    [
                        TrackingWheel::new(left_motors, Robot::WHEEL_DIAMETER, -5.75, None),
                        TrackingWheel::new(right_motors, Robot::WHEEL_DIAMETER, 5.75, None),
                    ],
                    [TrackingWheel::new(
                        enc,
                        Robot::TRACKING_WHEEL_DIAMETER,
                        Robot::SIDEWAYS_TRACKING_WHEEL_OFFSET,
                        None,
                    )],
                    Some(imu),
                ),
            )
        },

        // Intake
        intake: Intake::new(
            [Motor::new(
                peripherals.port_1,
                Gearset::Blue,
                Direction::Forward,
            )],
            [
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
            ],
            OpticalSensor::new(peripherals.port_15),
            AdiDigitalOut::new(peripherals.adi_e),
        ),

        // Lady Brown
        lady_brown: LadyBrown::new(
            [Motor::new(
                peripherals.port_3,
                Gearset::Green,
                Direction::Reverse,
            )],
            RotationSensor::new(peripherals.port_9, Direction::Forward),
            Robot::LADY_BROWN_PID,
        ),

        // Goal Clamp
        clamp: AdiDigitalOut::new(peripherals.adi_a),

        // Goal Rush Arms
        left_arm: AdiDigitalOut::new(peripherals.adi_c),
        right_arm: AdiDigitalOut::new(peripherals.adi_b),
        pinchers: AdiDigitalOut::new(peripherals.adi_d),
    };

    robot.compete().await;
}
