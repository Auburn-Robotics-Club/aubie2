extern crate alloc;

use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use evian::control::ControlLoop;
use vexide::{
    devices::smart::{Motor, RotationSensor},
    prelude::{sleep, spawn, Position, SmartDevice, Task},
};

use crate::hardware::Solenoid;

pub struct Overclock<S: Solenoid, F: ControlLoop<Input = f64, Output = f64> + 'static> {
    lift: S,
    pub flipper_feedback: Rc<RefCell<F>>,
    target: Rc<RefCell<Position>>,
    _task: Task<()>,
}

impl<S: Solenoid, F: ControlLoop<Input = f64, Output = f64> + 'static> Overclock<S, F> {
    pub fn new(
        lift: S,
        mut flipper: Motor,
        rotation_sensor: RotationSensor,
        flipper_feedback: F,
    ) -> Self {
        let target = Rc::new(RefCell::new(Position::from_degrees(70.0)));
        let flipper_feedback = Rc::new(RefCell::new(flipper_feedback));

        Overclock {
            lift,
            target: target.clone(),
            flipper_feedback: flipper_feedback.clone(),
            _task: spawn(async move {
                loop {
                    let voltage = match rotation_sensor.position() {
                        Ok(position) => flipper_feedback.borrow_mut().update(
                            target.borrow().as_degrees(),
                            position.as_degrees(),
                            Motor::UPDATE_INTERVAL,
                        ),
                        Err(err) => {
                            log::warn!("{err}");
                            0.0
                        }
                    };

                    _ = flipper.set_voltage(voltage);

                    sleep(Duration::from_millis(5)).await;
                }
            }),
        }
    }

    pub fn raise(&mut self) -> Result<(), S::Error> {
        self.lift.set_high()
    }

    pub fn lower(&mut self) -> Result<(), S::Error> {
        self.lift.set_low()
    }

    pub fn toggle(&mut self) -> Result<(), S::Error> {
        self.lift.toggle()
    }

    pub fn flip(&mut self) -> Result<(), S::Error> {
        self.lift.toggle()
    }

    pub fn set_target(&mut self, target: Position) {
        *self.target.borrow_mut() = target;
    }

    pub fn feedback(&self) -> Rc<RefCell<F>> {
        self.flipper_feedback.clone()
    }

    pub fn target(&self) -> Position {
        *self.target.borrow()
    }

    pub fn is_raised(&self) -> Result<bool, S::Error> {
        self.lift.is_high()
    }

    /// Score ring
    pub async fn score(&mut self) {
        self.set_target(Position::from_degrees(285.0));
        sleep(Duration::from_millis(800)).await;
        self.set_target(Position::from_degrees(70.0));
    }

    /// Score ring with intermediate position to position rings correctly in auto.
    pub async fn score_safe(&mut self) {
        self.set_target(Position::from_degrees(130.0));
        sleep(Duration::from_millis(1000)).await;
        self.set_target(Position::from_degrees(285.0));
        sleep(Duration::from_millis(800)).await;
        self.set_target(Position::from_degrees(70.0));
    }
}
