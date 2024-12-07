#![no_main]
#![no_std]

use core::time::Duration;

use aubie2::{hardware::SwitchBoard, theme::THEME_WAR_EAGLE};
use vexide::{devices::adi::digital::LogicLevel, prelude::*};

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    let mut switchboard = SwitchBoard::new(peripherals.port_1);

    sleep(Duration::from_millis(500)).await;

    loop {
        println!("High");
        switchboard.set_a(LogicLevel::High).unwrap();
        switchboard.set_b(LogicLevel::High).unwrap();
        switchboard.set_c(LogicLevel::High).unwrap();
        switchboard.set_d(LogicLevel::High).unwrap();
        sleep(Duration::from_secs(1)).await;

        println!("Low");
        switchboard.set_a(LogicLevel::Low).unwrap();
        switchboard.set_b(LogicLevel::Low).unwrap();
        switchboard.set_c(LogicLevel::Low).unwrap();
        switchboard.set_d(LogicLevel::Low).unwrap();
        sleep(Duration::from_secs(1)).await;
    }
}
