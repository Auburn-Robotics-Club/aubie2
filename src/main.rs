#![no_main]
#![no_std]

pub mod hardware;
pub mod subsystems;
pub mod theme;

use evian::control::pid::Pid;
use subsystems::{
    lady_brown::{LadyBrown, LadyBrownPosition, LadyBrownTarget},
    Drivetrain, Intake,
};
use theme::THEME_WAR_EAGLE;
use vexide::prelude::*;

pub struct Robot {
    pub controller: Controller,

    pub drivetrain: Drivetrain<4, 4>,
    pub intake: Intake<2>,

    pub lady_brown: LadyBrown<2, Pid>,
    pub lady_brown_state: LadyBrownTarget,

    pub clamp: AdiDigitalOut,
}

impl Compete for Robot {
    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self.drivetrain.arcade(
                state.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                state.left_stick.x() * Motor::V5_MAX_VOLTAGE,
            );

            if state.button_b.is_now_pressed() {
                self.lady_brown_state = match self.lady_brown_state {
                    LadyBrownTarget::Position(state) => match state {
                        LadyBrownPosition::Lowered => LadyBrownTarget::Position(LadyBrownPosition::Raised),
                        LadyBrownPosition::Raised => LadyBrownTarget::Position(LadyBrownPosition::Lowered),
                    },
                    LadyBrownTarget::Manual(_) => LadyBrownTarget::Position(LadyBrownPosition::Raised),
                };
            }

            if state.button_r1.is_pressed() {
                _ = self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if state.button_r2.is_pressed() {
                _ = self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                _ = self.intake.set_voltage(0.0);
            }

            if state.button_l1.is_pressed() {
                self.lady_brown_state =
                    LadyBrownTarget::Manual(MotorControl::Voltage(Motor::V5_MAX_VOLTAGE));
            } else if state.button_l2.is_pressed() {
                self.lady_brown_state =
                    LadyBrownTarget::Manual(MotorControl::Voltage(-Motor::V5_MAX_VOLTAGE));
            } else if let LadyBrownTarget::Manual(_) = self.lady_brown_state {
                self.lady_brown_state =
                    LadyBrownTarget::Manual(MotorControl::Brake(BrakeMode::Hold));
            }

            _ = self.lady_brown.update(self.lady_brown_state);

            if state.button_a.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            sleep(Motor::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            [
                Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
            ],
            [
                Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            ],
        ),
        intake: Intake::new([
            Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
        ]),
        lady_brown: LadyBrown::new(
            [
                Motor::new(peripherals.port_14, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_16, Gearset::Green, Direction::Reverse),
            ],
            RotationSensor::new(peripherals.port_13, Direction::Forward),
            Pid::new(0.45, 0.0, 0.001, None),
        ),
        lady_brown_state: LadyBrownTarget::default(),
        clamp: AdiDigitalOut::new(peripherals.adi_h),
    };

    robot.compete().await;
}
