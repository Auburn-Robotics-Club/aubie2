#![no_main]
#![no_std]

use core::time::Duration;

use aubie2::theme::THEME_WAR_EAGLE;
use vexide::{
    devices::{
        adi::digital::LogicLevel,
        smart::{
            serial::{SerialError, SerialPort},
            SmartPort,
        },
    },
    prelude::*,
};

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

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    let mut board = SwitchBoard::new(peripherals.port_14);

    sleep(Duration::from_millis(500)).await;

    loop {
        println!("High");
        board.set_a(LogicLevel::High).unwrap();
        board.set_b(LogicLevel::High).unwrap();
        board.set_c(LogicLevel::High).unwrap();
        board.set_d(LogicLevel::High).unwrap();
        sleep(Duration::from_secs(1)).await;

        println!("Low");
        board.set_a(LogicLevel::Low).unwrap();
        board.set_b(LogicLevel::Low).unwrap();
        board.set_c(LogicLevel::Low).unwrap();
        board.set_d(LogicLevel::Low).unwrap();
        sleep(Duration::from_secs(1)).await;
    }
}
