extern crate alloc;

use evian::control::Feedback;
use vexide::{
    devices::{
        position::Position,
        smart::{
            motor::{Motor, MotorError},
            RotationSensor, SmartDevice,
        },
    },
    prelude::MotorControl,
};

pub struct LadyBrown<const COUNT: usize, F: Feedback<Error = f64, Output = f64>> {
    pub motors: [Motor; COUNT],
    pub rotation_sensor: RotationSensor,
    pub feedback: F,
}

impl<const COUNT: usize, F: Feedback<Error = f64, Output = f64>> LadyBrown<COUNT, F> {
    pub fn new(motors: [Motor; COUNT], rotation_sensor: RotationSensor, feedback: F) -> Self {
        Self {
            motors,
            rotation_sensor,
            feedback,
        }
    }

    pub fn update(&mut self, target: LadyBrownTarget) -> Result<(), MotorError> {
        let motor_target = match target {
            LadyBrownTarget::State(state) => {
                let angle = self.rotation_sensor.position()?.as_degrees();

                let error = state.target_position().as_degrees() - angle;

                MotorControl::Voltage(self.feedback.update(error, Motor::UPDATE_INTERVAL))
            }
            LadyBrownTarget::Motor(v) => v,
        };

        for motor in self.motors.iter_mut() {
            motor.set_target(motor_target)?;
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LadyBrownTarget {
    State(LadyBrownState),
    Motor(MotorControl),
}

impl Default for LadyBrownTarget {
    fn default() -> Self {
        Self::State(LadyBrownState::Lowered)
    }
}

#[derive(Default, Debug, Clone, Copy, Eq, PartialEq)]
pub enum LadyBrownState {
    #[default]
    Lowered,
    Raised,
}

impl LadyBrownState {
    pub fn target_position(&self) -> Position {
        match self {
            Self::Raised => Position::from_degrees(240.0),
            Self::Lowered => Position::from_degrees(210.0),
        }
    }
}
