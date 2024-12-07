extern crate alloc;

use alloc::{rc::Rc, sync::Arc};
use core::{
    cell::RefCell,
    sync::atomic::{AtomicI32, Ordering},
    time::Duration,
};

use log::info;
use vexide::{
    core::time::Instant,
    prelude::{sleep, spawn, BrakeMode, Motor, OpticalSensor, SmartDevice, Task},
};

/// Intake Rejection Color
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum RejectColor {
    /// Reject blue rings
    Blue,

    /// Reject red rings
    Red,
}

/// Ring intake with color sorting capabilities.
pub struct Intake {
    _task: Task<()>,
    voltage: Arc<AtomicI32>,
    reject_color: Rc<RefCell<Option<RejectColor>>>,
}

impl Intake {
    pub fn new<const COUNT: usize>(
        mut motors: [Motor; COUNT],
        mut optical: OpticalSensor,
        reject_color: Option<RejectColor>,
    ) -> Self {
        let voltage = Arc::new(AtomicI32::new(0));
        let reject_color = Rc::new(RefCell::new(reject_color));

        Self {
            voltage: voltage.clone(),
            reject_color: reject_color.clone(),
            _task: spawn(async move {
                optical.set_led_brightness(100.0).unwrap();
                _ = optical.set_integration_time(Duration::from_millis(8));

                let mut rejecting = false;
                let mut reject_timestamp = Instant::now();

                loop {
                    let voltage = voltage.load(Ordering::Acquire) as f64;

                    if let Some(reject_color) = *reject_color.borrow() {
                        if let Ok(hue) = optical.hue() {
                            let matches_bad_ring_color = match reject_color {
                                RejectColor::Blue => (100.0..200.0).contains(&hue),
                                RejectColor::Red => todo!(),
                            };

                            if matches_bad_ring_color && !rejecting {
                                info!("Rejected ring with color {:?}.", reject_color);
                                reject_timestamp = Instant::now();
                                rejecting = true;
                            }
                        }
                    }

                    if (rejecting
                        && reject_timestamp.elapsed() > Duration::from_millis(75)
                        && reject_timestamp.elapsed() < Duration::from_millis(250))
                        && voltage > 0.0
                    {
                        for motor in motors.iter_mut() {
                            _ = motor.brake(BrakeMode::Hold);
                        }
                    } else {
                        if rejecting && reject_timestamp.elapsed() > Duration::from_millis(250) {
                            rejecting = false;
                        }

                        for motor in motors.iter_mut() {
                            _ = motor.set_voltage(voltage);
                        }
                    }

                    sleep(OpticalSensor::UPDATE_INTERVAL).await;
                }
            }),
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        self.voltage.store(voltage as i32, Ordering::Release);
    }

    pub fn set_reject_color(&mut self, reject_color: Option<RejectColor>) {
        *self.reject_color.borrow_mut() = reject_color;
    }
}
