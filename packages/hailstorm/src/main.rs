#![no_main]
#![no_std]

extern crate alloc;

use core::time::Duration;

use aubie2::{logger::SerialLogger, subsystems::overclock::Overclock, theme::THEME_WAR_EAGLE};
use evian::prelude::*;
use log::{error, info, LevelFilter};
use vexide::prelude::*;

static LOGGER: SerialLogger = SerialLogger;

pub struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, ParallelWheelTracking>,
    lift: Overclock,
    intake: [Motor; 2],
    clamp: AdiDigitalOut,
}

impl Compete for Robot {
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
                    _ = motor.set_voltage(Motor::V5_MAX_VOLTAGE);
                } else if state.button_down.is_pressed() {
                    _ = motor.set_voltage(-Motor::V5_MAX_VOLTAGE);
                } else {
                    _ = motor.set_voltage(0.0);
                }
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

    info!("Calibrating IMU");
    let mut imu = InertialSensor::new(peripherals.port_11);

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
                Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
            ];
            let right_motors = shared_motors![
                Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
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

        lift: todo!(),
        intake: todo!(),

        // Mogo
        clamp: AdiDigitalOut::new(peripherals.adi_h),
    };

    robot.compete().await;
}
