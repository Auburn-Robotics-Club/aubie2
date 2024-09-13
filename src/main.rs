#![no_main]
#![no_std]

extern crate alloc;

use alloc::{vec, vec::Vec};

use vexide::prelude::*;

/// Base robot struct
/// 
/// Holds all devices, peripherals, and subsytems on the robot.
struct Robot {
    controller: Controller,
    left_motors: Vec<Motor>,
    right_motors: Vec<Motor>,
}

impl Compete for Robot {
    /// Driver control routine
    async fn driver(&mut self) {
        loop {
            // Read controller joystick data. This is returned from [-1.0, 1.0].
            let fwd = self.controller.left_stick.y().unwrap() as f64;
            let turn = self.controller.left_stick.x().unwrap() as f64;

            // Set left/right motor voltages using controller readings.
            //
            // This uses the single-stick arcade control scheme, where the left stick's
            // y-axis controls linear (forward/back) movement and the x-axis controls
            // rotational movement.
            for motor in self.left_motors.iter_mut() {
                motor
                    .set_voltage((fwd + turn) * Motor::MAX_VOLTAGE)
                    .unwrap();
            }
            for motor in self.right_motors.iter_mut() {
                motor
                    .set_voltage((fwd - turn) * Motor::MAX_VOLTAGE)
                    .unwrap();
            }

            // Yield back to the async runtime to let other tasks run.
            sleep(Motor::DATA_WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        controller: peripherals.primary_controller,
        left_motors: vec![
            Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
        ],
        right_motors: vec![
            Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
        ],
    };

    robot.compete().await;
}
