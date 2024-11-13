#![no_main]
#![no_std]

mod subsystems;
mod control;

use vexide::prelude::*;

use control::pid::Pid;
use subsystems::{Drivetrain, Intake, LadyBrown};

pub struct Robot {
    pub controller: Controller,

    pub drivetrain: Drivetrain<4, 4>,
    pub intake: Intake<2>,
    pub lady_brown: LadyBrown<2, Pid>,

    pub clamp: AdiDigitalOut,
}

impl Compete for Robot {
    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self.drivetrain.arcade(state.left_stick.x(), state.left_stick.y());

            if state.button_b.is_pressed() {
                _ = self.intake.intake();
            } else if state.button_down.is_pressed() {
                _ = self.intake.outtake();
            } else {
                _ = self.intake.set_voltage(0.0);
            }

            if state.button_a.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            [
                Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            ],
            [
                Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
            ],
        ),
        intake: Intake::new([
            Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
        ]),
        lady_brown: LadyBrown::new(
            [
                Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_7, Gearset::Green, Direction::Reverse),
            ],
            RotationSensor::new(peripherals.port_8, Direction::Forward),
            Pid::new(1.5, 0.0, 0.5, None),
        ),
        clamp: AdiDigitalOut::new(peripherals.adi_a),
    };

    robot.compete().await;
}
