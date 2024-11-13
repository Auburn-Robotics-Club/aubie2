use vexide::devices::{
    position::Position,
    smart::{
        motor::{Motor, MotorError},
        RotationSensor,
    },
};

use crate::control::Feedback;

pub struct LadyBrown<const COUNT: usize, F: Feedback<Error = f64, Output = f64>> {
    motors: [Motor; COUNT],
    rotation_sensor: RotationSensor,
    feedback: F,
}

impl<const COUNT: usize, F: Feedback<Error = f64, Output = f64>> LadyBrown<COUNT, F> {
    pub fn new(motors: [Motor; COUNT], rotation_sensor: RotationSensor, feedback: F) -> Self {
        Self {
            motors,
            rotation_sensor,
            feedback,
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) -> Result<(), MotorError> {
        for motor in self.motors.iter_mut() {
            motor.set_voltage(voltage)?;
        }

        Ok(())
    }

    pub async fn rotate_to_position(&mut self, _position: Position) -> Result<(), MotorError> {
        todo!()
    }
}
