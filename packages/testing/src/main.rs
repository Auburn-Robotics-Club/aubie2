#![no_main]
#![no_std]

extern crate alloc;

use core::time::Duration;

use aubie2::theme::THEME_WAR_EAGLE;
use vex_sdk::v5_image;
use vexide::{devices::math::Point2, prelude::*};

pub struct Image {
    width: u32,
    height: u32,
    data: alloc::boxed::Box<[u32]>,
}

impl Image {
    pub fn decode_png(ibuf: &[u8], width: u32, height: u32) -> Self {
        unsafe {
            let mut data = alloc::vec![0; (width * height * 4) as usize].into_boxed_slice();
            let mut img: v5_image = v5_image {
                width: width as _,
                height: height as _,
                data: data.as_mut_ptr(),
                p: core::ptr::null_mut(),
            };

            if vex_sdk::vexImagePngRead(ibuf.as_ptr(), &mut img, width, height, ibuf.len() as _)
                != 1
            {
                panic!("Failed to decode PNG image.");
            }

            Image {
                width,
                height,
                data,
            }
        }
    }

    pub fn draw(&self, _: &mut Display, offset: Point2<i16>) {
        let x2 = offset.x + self.width as i16 - 1;
        let y2 = offset.y + self.height as i16 - 1;

        unsafe {
            vex_sdk::vexDisplayCopyRect(
                offset.x as _,
                (offset.y + Display::HEADER_HEIGHT) as i32,
                x2 as _,
                y2 as _,
                self.data.as_ptr().cast_mut(),
                (x2 - offset.x + 1) as _,
            );
        }
    }
}

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(mut peripherals: Peripherals) {
    let mut serial = SerialPort::open(peripherals.port_21, 9600).await;

    Image::decode_png(include_bytes!("./parky.png"), 480, 272)
        .draw(&mut peripherals.display, Point2 { x: 0, y: -32 });

    _ = serial.write_byte(0b11011010);

    loop {
        println!("{:?}", serial.unread_bytes().unwrap());

        sleep(Duration::from_millis(500)).await;
    }

    // otos.calibrate().await;
}
