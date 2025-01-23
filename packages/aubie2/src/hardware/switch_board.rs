//! AUBIE Solenoid SwitchBoard

extern crate alloc;

use alloc::rc::Rc;
use core::cell::RefCell;

use vexide::devices::{
    adi::digital::LogicLevel,
    smart::{
        serial::{SerialError, SerialPort},
        SmartPort,
    },
};

use super::Solenoid;

#[derive(Debug)]
pub struct SolenoidSwitch {
    shift: u8,
    state: Rc<RefCell<SwitchBoardState>>,
}

impl Solenoid for SolenoidSwitch {
    type Error = SerialError;

    fn set_level(&mut self, level: LogicLevel) -> Result<(), Self::Error> {
        self.state.borrow_mut().set(self.shift, level)
    }

    fn level(&self) -> Result<LogicLevel, Self::Error> {
        Ok(self.state.borrow().get(self.shift))
    }
}

/// Carson's Box of Fun
#[derive(Debug)]
pub struct SwitchBoard {
    pub solenoid_a: SolenoidSwitch,
    pub solenoid_b: SolenoidSwitch,
    pub solenoid_c: SolenoidSwitch,
    pub solenoid_d: SolenoidSwitch,
}

impl SwitchBoard {
    pub fn new(port: SmartPort) -> Self {
        let mut serial = SerialPort::open(port, 9600);

        let shared_state = Rc::new(RefCell::new(SwitchBoardState { serial, state: 0 }));

        Self {
            solenoid_a: SolenoidSwitch {
                state: shared_state.clone(),
                shift: 0,
            },
            solenoid_b: SolenoidSwitch {
                state: shared_state.clone(),
                shift: 1,
            },
            solenoid_c: SolenoidSwitch {
                state: shared_state.clone(),
                shift: 2,
            },
            solenoid_d: SolenoidSwitch {
                state: shared_state,
                shift: 3,
            },
        }
    }
}

#[derive(Debug)]
struct SwitchBoardState {
    state: u8,
    serial: SerialPort,
}

impl SwitchBoardState {
    fn set(&mut self, shift: u8, level: LogicLevel) -> Result<(), SerialError> {
        self.state = self.state & !(1 << shift) | ((level.is_high() as u8) << shift);
        log::debug!("Solenoid {:b}", self.state);
        self.serial.write_byte(self.state)
    }

    fn get(&self, shift: u8) -> LogicLevel {
        if (self.state & (1 << shift)) != 0 {
            LogicLevel::High
        } else {
            LogicLevel::Low
        }
    }
}
