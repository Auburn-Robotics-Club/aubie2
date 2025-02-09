#![no_main]
#![no_std]

extern crate alloc;

pub mod autons;

use core::time::Duration;

use alloc::format;
use aubie2::{logger::SerialLogger, subsystems::overclock::Overclock, theme::THEME_WAR_EAGLE};
use evian::{control::Pid, prelude::*};
use log::{error, info, LevelFilter};
use vexide::{
    core::time::Instant,
    devices::{
        display::{Font, FontFamily, FontSize, HAlign, Text, VAlign},
        math::Point2,
    },
    prelude::*,
};

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

        match autons::skills(self).await {
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
        let mut correction_macro_timestamp: Option<Instant> = None;

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
                    _ = motor.set_voltage(Motor::V5_MAX_VOLTAGE * 1.0);
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

            if state.button_r2.is_now_pressed() && correction_macro_timestamp.is_none() {
                correction_macro_timestamp = Some(Instant::now());
                self.lift.set_target(Position::from_degrees(130.0));
            } else if let Some(timestamp) = correction_macro_timestamp {
                if timestamp.elapsed() > Duration::from_millis(800) {
                    self.lift.set_target(Position::from_degrees(70.0));
                    correction_macro_timestamp = None;
                }
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
    imu.calibrate().await.unwrap();
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
