use evian::control::ControlLoop;
use vexide::{devices::{
    adi::AdiDigitalOut,
    smart::{Motor, RotationSensor},
}, prelude::{spawn, Task}};

pub struct Overclock {
    lift: [AdiDigitalOut; 2],
    _task: Task<()>,
}

impl Overclock {
    pub fn new<F: ControlLoop<Input = f64, Output = f64>>(
        lift: [AdiDigitalOut; 2],
        _dunker: Motor,
        _rotation_sensor: RotationSensor,
        _dunker_feedback: F,
    ) -> Self {
        Overclock {
            lift,
            _task: spawn(async move {}),
        }
    }

    pub fn raise(&mut self) {
        // Error will never be returned since we are using onboard ADI.
        _ = self.lift[0].set_high();
        _ = self.lift[1].set_high();
    }

    pub fn lower(&mut self) {
        // Error will never be returned since we are using onboard ADI.
        _ = self.lift[0].set_low();
        _ = self.lift[1].set_low();
    }
}
