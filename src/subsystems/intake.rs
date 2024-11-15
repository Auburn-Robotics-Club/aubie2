use vexide::{devices::smart::motor::MotorError, prelude::Motor};

pub struct Intake<const COUNT: usize> {
    motors: [Motor; COUNT],
}

impl<const COUNT: usize> Intake<COUNT> {
    pub fn new(motors: [Motor; COUNT]) -> Self {
        Self { motors }
    }

    pub fn set_voltage(&mut self, voltage: f64) -> Result<(), MotorError> {
        for motor in self.motors.iter_mut() {
            motor.set_voltage(voltage)?;
        }

        Ok(())
    }
}
