use vexide::{devices::smart::motor::MotorError, prelude::Motor};

pub struct Drivetrain<const LEFT_COUNT: usize, const RIGHT_COUNT: usize> {
    left_motors: [Motor; LEFT_COUNT],
    right_motors: [Motor; RIGHT_COUNT],
}

impl<const LEFT_COUNT: usize, const RIGHT_COUNT: usize> Drivetrain<LEFT_COUNT, RIGHT_COUNT> {
    pub fn new(left_motors: [Motor; LEFT_COUNT], right_motors: [Motor; RIGHT_COUNT]) -> Self {
        Self { left_motors, right_motors }
    }

    pub fn tank(&mut self, left_voltage: f64, right_voltage: f64) -> Result<(), MotorError> {
        for motor in self.left_motors.iter_mut() {
            motor.set_voltage(left_voltage)?;
        }

        for motor in self.right_motors.iter_mut() {
            motor.set_voltage(right_voltage)?;
        }
        
        Ok(())
    }

    pub fn arcade(&mut self, forward_voltage: f64, turn_voltage: f64) -> Result<(), MotorError> {
        for motor in self.left_motors.iter_mut() {
            motor.set_voltage(forward_voltage + turn_voltage)?;
        }

        for motor in self.right_motors.iter_mut() {
            motor.set_voltage(forward_voltage - turn_voltage)?;
        }
        
        Ok(())
    }
}