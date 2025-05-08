#![no_main]
#![no_std]

extern crate alloc;

pub mod routes;

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
        LadyBrownTarget::Position(Position::from_degrees(190.0));
    pub const LADY_BROWN_RAISED: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(150.0));
    pub const LADY_BROWN_UP: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(35.0));
    pub const LADY_BROWN_SCORED: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(25.0));
    pub const LADY_BROWN_FLAT: LadyBrownTarget =
        LadyBrownTarget::Position(Position::from_degrees(-10.0));

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

        self.blue().await;

        info!("Route completed successfully in {:?}.", start.elapsed());
        info!(
            "Position: {}\nHeading: {}° ({}rad)",
            self.drivetrain.tracking.position(),
            self.drivetrain.tracking.heading().as_degrees(),
            self.drivetrain.tracking.heading().as_radians()
        );
    }

    async fn driver(&mut self) {
        self.intake.set_reject_color(None);

        loop {
            let state = self.controller.state().unwrap_or_default();

            // Single-stick arcade joystick control
            _ = self.drivetrain.motors.set_voltages(
                Voltages::from_arcade(
                    state.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                    state.left_stick.x() * Motor::V5_MAX_VOLTAGE,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );

            // Raise/lower ladybrown when B is pressed.
            if state.button_b.is_now_pressed() {
                self.lady_brown.set_target(match self.lady_brown.target() {
                    Self::LADY_BROWN_LOWERED | LadyBrownTarget::Manual(_) => {
                        Self::LADY_BROWN_RAISED
                    }
                    _ => Self::LADY_BROWN_LOWERED,
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

            if state.button_x.is_now_pressed() {
                _ = self.right_arm.toggle();
            }
            if state.button_up.is_now_pressed() {
                _ = self.left_arm.toggle();
            }
            if state.button_right.is_now_pressed() {
                _ = self.pinchers.toggle();
            }

            if state.button_y.is_now_pressed() {
                self.lady_brown.set_target(Self::LADY_BROWN_SCORED);
            }

            // A to toggle mogo mech.
            if state.button_a.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            sleep(Motor::UPDATE_INTERVAL).await;
        }
    }
}

// MARK: Main

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    SerialLogger.init(LevelFilter::Trace).unwrap();

    let enc = CustomEncoder::<8192>::new(peripherals.adi_g, peripherals.adi_h, Direction::Forward);
    let mut display = peripherals.display;
    let mut imu = InertialSensor::new(peripherals.port_9);
    let mut controller = peripherals.primary_controller;

    calibrate_imu(&mut controller, &mut display, &mut imu).await;

    let robot = Robot {
        // Controller
        controller,

        // Drivetrain Model & Localization
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

            // Drivetrain Model
            Drivetrain::new(
                Differential::from_shared(left_motors.clone(), right_motors.clone()),
                WheeledTracking::new(
                    Vec2::new(0.0, 0.0),
                    90.0.deg(),
                    [
                        TrackingWheel::new(left_motors.clone(), 3.25, -5.75, Some(36.0 / 48.0)),
                        TrackingWheel::new(right_motors.clone(), 3.25, 5.75, Some(36.0 / 48.0)),
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
                peripherals.port_15,
                Gearset::Blue,
                Direction::Forward,
            )],
            [
                Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
            ],
            OpticalSensor::new(peripherals.port_21),
            AdiDigitalOut::new(peripherals.adi_d),
        ),

        // Lady Brown Arm
        lady_brown: LadyBrown::new(
            [Motor::new(
                peripherals.port_2,
                Gearset::Green,
                Direction::Reverse,
            )],
            RotationSensor::new(peripherals.port_8, Direction::Forward),
            Pid::new(0.19, 0.0, 0.01, None),
        ),

        // Mogo
        clamp: AdiDigitalOut::new(peripherals.adi_f),

        // Goal Rush Arms
        left_arm: AdiDigitalOut::new(peripherals.adi_e),
        right_arm: AdiDigitalOut::new(peripherals.adi_a),
        pinchers: AdiDigitalOut::new(peripherals.adi_b),
    };

    robot.compete().await;
}
