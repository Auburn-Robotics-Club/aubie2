use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use evian::control::loops::Feedback;
use log::warn;
use vexide::{
    devices::{
        position::Position,
        smart::{
            motor::{Motor, MotorControl},
            RotationSensor, SmartDevice,
        },
    },
    task::{spawn, Task},
    time::sleep,
};

/// Lady brown wallstake mechanism.
pub struct LadyBrown {
    target: Rc<RefCell<LadyBrownTarget>>,
    _task: Task<()>,
}

impl LadyBrown {
    pub fn new<const COUNT: usize, F: Feedback<Input = f64, Output = f64> + 'static>(
        mut motors: [Motor; COUNT],
        rotation_sensor: RotationSensor,
        mut feedback: F,
    ) -> Self {
        let target = Rc::new(RefCell::new(LadyBrownTarget::Manual(
            MotorControl::Voltage(0.0),
        )));

        Self {
            target: target.clone(),
            _task: spawn(async move {
                loop {
                    // debug!("{:?}", rotation_sensor.position().unwrap().as_degrees());
                    match rotation_sensor.position() {
                        Ok(position) => {
                            let motor_target = match *target.borrow() {
                                LadyBrownTarget::Position(state) => {
                                    MotorControl::Voltage(feedback.update(
                                        state.as_degrees(),
                                        position.as_degrees(),
                                        Motor::UPDATE_INTERVAL,
                                    ))
                                }
                                LadyBrownTarget::Manual(v) => v,
                            };

                            for motor in motors.iter_mut() {
                                _ = motor.set_target(motor_target);
                            }
                        }
                        Err(err) => {
                            warn!("{err}");
                        }
                    }

                    sleep(Duration::from_millis(5)).await;
                }
            }),
        }
    }

    pub fn set_target(&mut self, target: LadyBrownTarget) {
        *self.target.borrow_mut() = target;
    }

    pub fn target(&self) -> LadyBrownTarget {
        *self.target.borrow()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LadyBrownTarget {
    Position(Position),
    Manual(MotorControl),
}
