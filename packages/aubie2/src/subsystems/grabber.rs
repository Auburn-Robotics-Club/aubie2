use crate::hardware::Solenoid;

pub struct Grabber<P: Solenoid, E: Solenoid> {
    pincher: P,
    extender: E,
}

impl<P: Solenoid, E: Solenoid> Grabber<P, E> {
    pub fn new(
        pincher: P,
        extender: E,
    ) -> Self {
        Self {
            pincher,
            extender,
        }
    }

    pub fn extend(&mut self) -> Result<(), E::Error> {
        self.extender.set_high()
    }

    pub fn retract(&mut self) -> Result<(), E::Error> {
        self.extender.set_low()
    }

    pub fn pinch(&mut self) -> Result<(), P::Error> {
        self.pincher.set_high()
    }

    pub fn release(&mut self) -> Result<(), P::Error> {
        self.pincher.set_low()
    }

    pub fn toggle_pincher(&mut self) -> Result<(), P::Error> {
        self.pincher.toggle()
    }

    pub fn toggle_extender(&mut self) -> Result<(), E::Error> {
        self.extender.toggle()
    }
}
