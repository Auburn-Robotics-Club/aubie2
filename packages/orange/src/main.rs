#![no_main]
#![no_std]

extern crate alloc;

pub mod autons;

use alloc::format;

use aubie2::{
    logger::SerialLogger,
    subsystems::{
        lady_brown::{LadyBrown, LadyBrownTarget},
        Grabber, Intake,
    },
    theme::THEME_WAR_EAGLE,
};
use evian::{control::loops::Pid, prelude::*};
use log::{info, LevelFilter};
use vexide::{
    devices::{
        display::{Font, FontFamily, FontSize, HAlign, Text, VAlign},
        math::Point2,
    },
    prelude::*,
    time::Instant,
};

pub const LADY_BROWN_LOWERED: Position = Position::from_degrees(190.0);
pub const LADY_BROWN_RAISED: Position = Position::from_degrees(154.0);
pub const LADY_BROWN_SCORED: Position = Position::from_degrees(35.0);

static LOGGER: SerialLogger = SerialLogger;

pub struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, WheeledTracking>,
    intake: Intake,
    lady_brown: LadyBrown,
    clamp: AdiDigitalOut,
    intake_raiser: AdiDigitalOut,
    grabber: Grabber,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let start = Instant::now();

        // match autons::red(self).await {
        //     Ok(()) => {
        //         info!("Route completed successfully in {:?}.", start.elapsed());
        //     }
        //     Err(err) => {
        //         error!(
        //             "Route encountered error after {:?}: {}",
        //             start.elapsed(),
        //             err
        //         );
        //     }
        // }

        // Dump tracking info to get ending pose of robot.
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
                    state.left_stick.x() * Motor::V5_MAX_VOLTAGE,
                    state.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );

            // Raise/lower ladybrown when B is pressed.
            if state.button_b.is_now_pressed() {
                self.lady_brown.set_target(match self.lady_brown.target() {
                    LadyBrownTarget::Position(state) => match state {
                        LADY_BROWN_LOWERED => LadyBrownTarget::Position(LADY_BROWN_RAISED),
                        _ => LadyBrownTarget::Position(LADY_BROWN_LOWERED),
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

            if state.button_x.is_now_pressed() {
                _ = self.grabber.toggle_extender();
            }

            if state.button_y.is_now_pressed() {
                self.lady_brown.set_target(LadyBrownTarget::Position(LADY_BROWN_SCORED));
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
    LOGGER.init(LevelFilter::Trace).unwrap();

    let mut display = peripherals.display;
    let mut imu = InertialSensor::new(peripherals.port_21);

    info!("Calibrating IMU");
    let imu_calibration_start = Instant::now();
    // imu.calibrate().await.unwrap();
    let imu_calibration_elapsed = imu_calibration_start.elapsed();
    info!("Calibration completed in {:?}.", imu_calibration_elapsed);

    display.draw_text(
        &Text::new_aligned(
            &format!("{:?}", imu_calibration_elapsed),
            Font::new(FontSize::LARGE, FontFamily::Monospace),
            Point2 {
                x: Display::HORIZONTAL_RESOLUTION / 2,
                y: Display::VERTICAL_RESOLUTION / 2,
            },
            HAlign::Center,
            VAlign::Center,
        ),
        Rgb::new(255, 255, 255),
        None,
    );

    let robot = Robot {
        // Controller
        controller: peripherals.primary_controller,

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
                Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
            ];

            // Drivetrain Model
            Drivetrain::new(
                Differential::from_shared(left_motors.clone(), right_motors.clone()),
                WheeledTracking::forward_only(
                    Vec2::new(0.0, 0.0),
                    270.0.deg(),
                    [
                        TrackingWheel::new(left_motors.clone(), 3.25, -5.75, Some(36.0 / 48.0)),
                        TrackingWheel::new(right_motors.clone(), 3.25, 5.75, Some(36.0 / 48.0)),
                    ],
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
                Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
            ],
            OpticalSensor::new(peripherals.port_5),
            None,
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
        clamp: AdiDigitalOut::new(peripherals.adi_h),

        grabber: Grabber::new(
            AdiDigitalOut::new(peripherals.adi_g),
            AdiDigitalOut::new(peripherals.adi_a),
        ),

        intake_raiser: AdiDigitalOut::new(peripherals.adi_e),
    };

    robot.compete().await;
}
