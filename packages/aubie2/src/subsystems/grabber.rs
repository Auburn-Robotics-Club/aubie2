use vexide::{devices::PortError, prelude::AdiDigitalOut};

pub struct Grabber {
    pincher: AdiDigitalOut,
    extender: AdiDigitalOut,
}

impl Grabber {
    pub fn new(pincher: AdiDigitalOut, extender: AdiDigitalOut) -> Self {
        Self { pincher, extender }
    }

    pub fn extend(&mut self) -> Result<(), PortError> {
        self.extender.set_high()
    }

    pub fn retract(&mut self) -> Result<(), PortError> {
        self.extender.set_low()
    }

    pub fn pinch(&mut self) -> Result<(), PortError> {
        self.pincher.set_high()
    }

    pub fn release(&mut self) -> Result<(), PortError> {
        self.pincher.set_low()
    }

    pub fn toggle_pincher(&mut self) -> Result<(), PortError> {
        self.pincher.toggle()
    }

    pub fn toggle_extender(&mut self) -> Result<(), PortError> {
        self.extender.toggle()
    }
}
