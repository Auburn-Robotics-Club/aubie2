use alloc::format;

use log::info;
use vexide::{
    devices::display::{Font, FontFamily, FontSize, HAlign, Text, VAlign},
    prelude::{Display, InertialSensor, Rgb},
    time::Instant,
};

pub async fn calibrate_imu(display: &mut Display, imu: &mut InertialSensor) {
    info!("Calibrating IMU");

    let imu_calibration_start = Instant::now();
    imu.calibrate().await.unwrap();
    let imu_calibration_elapsed = imu_calibration_start.elapsed();
    info!("Calibration completed in {:?}.", imu_calibration_elapsed);

    display.draw_text(
        &Text::new_aligned(
            &format!("{:?}", imu_calibration_elapsed),
            Font::new(FontSize::LARGE, FontFamily::Monospace),
            [
                Display::HORIZONTAL_RESOLUTION / 2,
                Display::VERTICAL_RESOLUTION / 2,
            ],
            HAlign::Center,
            VAlign::Center,
        ),
        Rgb::new(255, 255, 255),
        None,
    );
}
