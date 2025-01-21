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
                _ = optical.set_integration_time(OpticalSensor::MIN_INTEGRATION_TIME);
                // _ = optical.set_led_brightness(1.0);

                let mut rejecting = false;
                let mut reject_timestamp = Instant::now();
                let mut prox_timestamp = Instant::now();
                let mut in_prox = false;

                loop {
                    let voltage = voltage.load(Ordering::Acquire) as f64;

                    if let Some(reject_color) = *reject_color.borrow() {
                        if let Ok(prox) = optical.proximity() {
                            if prox > 0.5 && !in_prox {
                                prox_timestamp = Instant::now();
                                in_prox = true;
                            }

                            if in_prox && prox_timestamp.elapsed() > Duration::from_millis(15) {
                                in_prox = false;
                            }
                        }

                        if in_prox {
                            if let Ok(hue) = optical.hue() {
                                log::debug!("hue {}", hue);
                                // let matches_bad_ring_color = in_prox
                                //     && match reject_color {
                                //         RejectColor::Blue => (80.0..250.0).contains(&hue),
                                //         RejectColor::Red => todo!(),
                                //     };

                                // if matches_bad_ring_color && !rejecting {
                                //     info!("Rejected {:?} ring with hue {}.", reject_color, hue);
                                //     reject_timestamp = Instant::now();
                                //     rejecting = true;
                                // }
                            }
                        }
                    }

                    let reject_elapsed = reject_timestamp.elapsed();

                    if rejecting
                        && reject_elapsed > Duration::from_millis(80)
                        && reject_elapsed < Duration::from_millis(200)
                        && voltage > 0.0
                    {
                        for motor in motors.iter_mut() {
                            _ = motor.brake(BrakeMode::Hold);
                        }
                    } else {
                        if rejecting && reject_elapsed > Duration::from_millis(200) {
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
