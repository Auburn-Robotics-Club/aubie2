//! AUBIE Solenoid SwitchBoard

use vexide::devices::{
    adi::digital::LogicLevel,
    smart::{
        serial::{SerialError, SerialPort},
        SmartPort,
    },
};

/// Carson's Box of Fun
#[derive(Debug, PartialEq)]
pub struct SwitchBoard {
    serial: SerialPort,
    state: u8,
}

impl SwitchBoard {
    pub fn new(port: SmartPort) -> Self {
        Self {
            serial: SerialPort::open(port, 9600),
            state: 0,
        }
    }

    fn set(&mut self, shift: u8, level: LogicLevel) -> Result<(), SerialError> {
        self.state = self.state & !(1 << shift) | ((level.is_high() as u8) << shift);
        self.serial.write_byte(self.state)
    }

    pub fn set_a(&mut self, level: LogicLevel) -> Result<(), SerialError> {
        self.set(0, level)
    }

    pub fn set_b(&mut self, level: LogicLevel) -> Result<(), SerialError> {
        self.set(1, level)
    }

    pub fn set_c(&mut self, level: LogicLevel) -> Result<(), SerialError> {
        self.set(2, level)
    }

    pub fn set_d(&mut self, level: LogicLevel) -> Result<(), SerialError> {
        self.set(3, level)
    }
}
