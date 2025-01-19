#![no_main]
#![no_std]

use core::time::Duration;

use aubie2::{hardware::{Solenoid, SwitchBoard}, theme::THEME_WAR_EAGLE};
use vexide::prelude::*;

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    let mut board = SwitchBoard::new(peripherals.port_1);

    sleep(Duration::from_millis(500)).await;

    loop {
        println!("High");
        board.solenoid_a.set_high().unwrap();
        board.solenoid_b.set_high().unwrap();
        board.solenoid_c.set_high().unwrap();
        board.solenoid_d.set_high().unwrap();
        sleep(Duration::from_secs(1)).await;

        println!("Low");
        board.solenoid_a.set_low().unwrap();
        board.solenoid_b.set_low().unwrap();
        board.solenoid_c.set_low().unwrap();
        board.solenoid_d.set_low().unwrap();
        sleep(Duration::from_secs(1)).await;
    }
}
