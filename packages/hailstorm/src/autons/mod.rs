use core::{f64::consts::PI, time::Duration};

use evian::control::{AngularPid, Pid, Tolerances};

mod red;
mod blue;
mod testing;
mod skills;

pub use red::red;
pub use blue::blue;
pub use testing::testing;
pub use skills::skills;

const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);

const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(4.0)
    .velocity_tolerance(0.25)
    .tolerance_duration(Duration::from_millis(15))
    .timeout(Duration::from_secs(10));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(8.0 * (PI / 180.0))
    .velocity_tolerance(0.09)
    .tolerance_duration(Duration::from_millis(15))
    .timeout(Duration::from_secs(10));
