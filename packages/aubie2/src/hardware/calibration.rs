use alloc::format;

use log::{error, info};
use vexide::{
    devices::display::{Font, FontFamily, FontSize, HAlign, Text, VAlign},
    prelude::{Controller, Display, InertialSensor, Rgb},
    time::Instant,
};

pub async fn calibrate_imu(
    controller: &mut Controller,
    display: &mut Display,
    imu: &mut InertialSensor,
) {
    info!("Calibrating IMU");
    _ = controller
        .screen
        .try_set_text("Calibrating...", 1, 1);
    let imu_calibration_start = Instant::now();
    
    if imu.calibrate().await.is_err() {
        error!("Calibration fail!");
        _ = controller
            .screen
            .try_set_text("Calibration fail!    ", 1, 1);
        return;
    }

    let imu_calibration_elapsed = imu_calibration_start.elapsed();

    info!("Calibration completed in {:?}.", imu_calibration_elapsed);

    _ = controller
        .screen
        .try_set_text(format!("{:?}    ", imu_calibration_elapsed), 1, 1);

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
