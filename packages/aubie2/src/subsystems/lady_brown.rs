use evian::control::ControlLoop;
use vexide::devices::{
    position::Position,
    smart::{
        motor::{Motor, MotorControl, MotorError},
        RotationSensor, SmartDevice,
    },
};

pub struct LadyBrown<const COUNT: usize, F: ControlLoop<Input = f64, Output = f64>> {
    pub motors: [Motor; COUNT],
    pub rotation_sensor: RotationSensor,
    pub feedback: F,
}

impl<const COUNT: usize, F: ControlLoop<Input = f64, Output = f64>> LadyBrown<COUNT, F> {
    pub fn new(motors: [Motor; COUNT], rotation_sensor: RotationSensor, feedback: F) -> Self {
        Self {
            motors,
            rotation_sensor,
            feedback,
        }
    }

    pub fn update(&mut self, target: LadyBrownTarget) -> Result<(), MotorError> {
        let motor_target = match target {
            LadyBrownTarget::Position(state) => MotorControl::Voltage(self.feedback.update(
                state.as_degrees(),
                self.rotation_sensor.position()?.as_degrees(),
                Motor::UPDATE_INTERVAL,
            )),
            LadyBrownTarget::Manual(v) => v,
        };

        for motor in self.motors.iter_mut() {
            motor.set_target(motor_target)?;
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LadyBrownTarget {
    Position(Position),
    Manual(MotorControl),
}
