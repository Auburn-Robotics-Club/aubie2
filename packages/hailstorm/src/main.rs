#![no_main]
#![no_std]

extern crate alloc;

pub mod autons;

use core::time::Duration;

use aubie2::{logger::SerialLogger, subsystems::overclock::Overclock, theme::THEME_WAR_EAGLE};
use evian::{control::Pid, prelude::*};
use log::{error, info, LevelFilter};
use vexide::{core::time::Instant, prelude::*};

static LOGGER: SerialLogger = SerialLogger;

pub struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, ParallelWheelTracking>,
    clamp: AdiDigitalOut,
    doinker: AdiDigitalOut,
    #[allow(unused)]
    lift: Overclock<AdiDigitalOut, Pid>,
    intake: [Motor; 2],
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let start = Instant::now();

        match autons::red_left(self).await {
            Ok(()) => {
                info!("Route completed successfully in {:?}.", start.elapsed());
            }
            Err(err) => {
                error!(
                    "Route encountered error after {:?}: {}",
                    start.elapsed(),
                    err
                );
            }
        }

        // Dump tracking info to get ending pose of robot.
        info!(
            "Position: {}\nHeading: {}° ({}rad)",
            self.drivetrain.tracking.position(),
            self.drivetrain.tracking.heading().as_degrees(),
            self.drivetrain.tracking.heading().as_radians()
        );
    }

    async fn driver(&mut self) {
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

            // Intake control
            for motor in self.intake.iter_mut() {
                if state.button_b.is_pressed() {
                    _ = motor.set_voltage(Motor::V5_MAX_VOLTAGE * 0.9);
                } else if state.button_down.is_pressed() {
                    _ = motor.set_voltage(-Motor::V5_MAX_VOLTAGE * 1.0);
                } else {
                    _ = motor.set_voltage(0.0);
                }
            }

            // A to toggle mogo mech.
            if state.button_a.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            if state.button_r1.is_now_pressed() {
                if self.lift.target() == Position::from_degrees(70.0) {
                    self.lift.set_target(Position::from_degrees(290.0));
                } else {
                    self.lift.set_target(Position::from_degrees(70.0));

                    if self.lift.is_raised().unwrap_or_default() {
                        _ = self.lift.lower();
                    }
                }
            }

            if state.button_l1.is_now_pressed() {
                _ = self.doinker.toggle();
            }

            if state.right_stick.y() < -0.7 {
                _ = self.lift.raise();
            } else if state.right_stick.y() > 0.7 {
                _ = self.lift.lower();
            }

            sleep(Motor::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    LOGGER.init(LevelFilter::Trace).unwrap();

    info!("Calibrating IMU");
    let mut imu = InertialSensor::new(peripherals.port_21);

    if let Err(err) = imu.calibrate().await {
        error!("IMU Calibration failed: {err}. Retrying...");
        if let Err(err) = imu.calibrate().await {
            error!("IMU Calibration failed again: {err}. Waiting 3 seconds...");
            sleep(Duration::from_secs(3)).await;
        }
    }

    info!("Calibration complete.");

    let robot = Robot {
        // Controller
        controller: peripherals.primary_controller,

        // Drivetrain Model & Localization
        drivetrain: {
            // Left/right motors shared between drivetrain and odometry.
            let left_motors = shared_motors![
                Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
            ];
            let right_motors = shared_motors![
                Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
            ];

            // Drivetrain Model
            Drivetrain::new(
                Differential::new(left_motors.clone(), right_motors.clone()),
                ParallelWheelTracking::new(
                    Vec2::new(0.0, 0.0),
                    270.0.deg(),
                    TrackingWheel::new(left_motors.clone(), 3.25, 5.75, Some(36.0 / 48.0)),
                    TrackingWheel::new(right_motors.clone(), 3.25, 5.75, Some(36.0 / 48.0)),
                    Some(imu),
                ),
            )
        },

        intake: [
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
        ],

        lift: Overclock::new(
            AdiDigitalOut::new(peripherals.adi_h),
            Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
            RotationSensor::new(peripherals.port_3, Direction::Forward),
            Pid::new(0.45, 0.0, 0.001, None),
        ),

        // Mogo clamp
        // dohickey c
        clamp: AdiDigitalOut::new(peripherals.adi_f),
        doinker: AdiDigitalOut::new(peripherals.adi_e),
    };

    robot.compete().await;
}
