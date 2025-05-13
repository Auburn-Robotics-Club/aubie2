extern crate alloc;

use alloc::{rc::Rc, sync::Arc};
use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicI32, Ordering},
    time::Duration,
};

use log::info;
use vexide::{
    devices::PortError, io::println, prelude::{sleep, spawn, AdiDigitalOut, BrakeMode, Motor, OpticalSensor, SmartDevice, Task}, time::Instant
};

/// Intake Rejection Color
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum RingColor {
    /// Reject blue rings
    Blue,

    /// Reject red rings
    Red,
}

/// Ring intake with color sorting capabilities.
pub struct Intake {
    _task: Task<()>,
    pub raiser: AdiDigitalOut,
    enable_jam: Arc<AtomicBool>,
    top_voltage: Arc<AtomicI32>,
    bottom_voltage: Arc<AtomicI32>,
    reject_color: Rc<RefCell<Option<RingColor>>>,
}

impl Intake {
    pub fn new<const BOTTOM_COUNT: usize, const TOP_COUNT: usize>(
        mut bottom_motors: [Motor; TOP_COUNT],
        mut top_motors: [Motor; BOTTOM_COUNT],
        mut optical: OpticalSensor,
        raiser: AdiDigitalOut,
    ) -> Self {
        let top_voltage = Arc::new(AtomicI32::new(0));
        let bottom_voltage = Arc::new(AtomicI32::new(0));
        let reject_color = Rc::new(RefCell::new(None));
        let enable_jam = Arc::new(AtomicBool::new(false));

        Self {
            top_voltage: top_voltage.clone(),
            bottom_voltage: bottom_voltage.clone(),
            reject_color: reject_color.clone(),
            enable_jam: enable_jam.clone(),
            raiser,
            _task: spawn(async move {
                _ = optical.set_integration_time(Duration::from_millis(4));
                // _ = optical.set_led_brightness(1.0);

                let mut rejecting = false;
                let mut reject_timestamp = Instant::now();
                let mut prox_timestamp = Instant::now();
                let mut in_prox = false;

                let mut jam_timestamp = Instant::now();
                let mut jammed = false;

                loop {
                    let top_voltage = top_voltage.load(Ordering::Acquire) as f64;
                    let bottom_voltage = bottom_voltage.load(Ordering::Acquire) as f64;

                    if let Some(reject_color) = *reject_color.borrow() {
                        if let Ok(prox) = optical.proximity() {
                            if prox > 0.3 && !in_prox {
                                prox_timestamp = Instant::now();
                                in_prox = true;
                            }

                            if in_prox && prox_timestamp.elapsed() > Duration::from_millis(20) {
                                in_prox = false;
                            }
                        }

                        if in_prox {
                            if let Ok(hue) = optical.hue() {
                                let matches_bad_ring_color = in_prox
                                    && match reject_color {
                                        RingColor::Blue => (55.0..250.0).contains(&hue),
                                        RingColor::Red => {
                                            (10.0..40.0).contains(&hue)
                                                || (338.0..360.0).contains(&hue)
                                        }
                                    };

                                if matches_bad_ring_color && !rejecting {
                                    info!("Rejected {:?} ring with hue {}.", reject_color, hue);
                                    reject_timestamp = Instant::now();
                                    rejecting = true;
                                }
                            }
                        }
                    }

                    let reject_elapsed = reject_timestamp.elapsed();

                    if rejecting
                        && reject_elapsed > Duration::from_millis(150)
                        && reject_elapsed < Duration::from_millis(300)
                    {
                        if top_voltage > 0.0 {
                            for motor in top_motors.iter_mut() {
                                _ = motor.brake(BrakeMode::Hold);
                            }
                        }
                    } else {
                        if rejecting && reject_elapsed > Duration::from_millis(200) {
                            rejecting = false;
                        }

                        for motor in top_motors.iter_mut() {
                            _ = motor.set_voltage(top_voltage);
                        }
                    }

                    if enable_jam.load(Ordering::Acquire) {
                        let avg_top_torque = {
                            let mut sum = 0.0;
                            let mut total = 0;
        
                            for motor in &top_motors {
                                if let Ok(torque) = motor.torque() {
                                    sum += torque;
                                    total += 1;
                                }
                            }
        
                            if total > 0 {
                                sum / (total as f64)
                            } else {
                                0.0
                            }
                        };
    
                        if !jammed && avg_top_torque < 0.25 {
                            jam_timestamp = Instant::now();
                        } else if jammed {
                            for motor in top_motors.iter_mut() {
                                _ = motor.set_voltage(-12.0);
                            }
                        }
    
                        if !jammed && jam_timestamp.elapsed() > Duration::from_millis(500) {
                            jammed = true;
                        } else if jammed && jam_timestamp.elapsed() > Duration::from_millis(1000) {
                            jammed = false;
                        }
                    }

                    for motor in bottom_motors.iter_mut() {
                        _ = motor.set_voltage(bottom_voltage);
                    }

                    sleep(OpticalSensor::UPDATE_INTERVAL).await;
                }
            }),
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        self.set_top_voltage(voltage);
        self.set_bottom_voltage(voltage);
    }

    pub fn set_top_voltage(&mut self, voltage: f64) {
        self.top_voltage.store(voltage as i32, Ordering::Release);
    }

    pub fn set_bottom_voltage(&mut self, voltage: f64) {
        self.bottom_voltage.store(voltage as i32, Ordering::Release);
    }

    pub fn set_reject_color(&mut self, reject_color: Option<RingColor>) {
        *self.reject_color.borrow_mut() = reject_color;
    }

    pub fn raise(&mut self) -> Result<(), PortError> {
        self.raiser.set_high()
    }

    pub fn lower(&mut self) -> Result<(), PortError> {
        self.raiser.set_low()
    }

    pub fn is_raised(&mut self) -> Result<bool, PortError> {
        self.raiser.is_high()
    }

    pub fn enable_jam_prevention(&mut self) {
        self.enable_jam.store(true, Ordering::Release);
    }

    pub fn disable_jam_prevention(&mut self) {
        self.enable_jam.store(false, Ordering::Release);
    }
}
