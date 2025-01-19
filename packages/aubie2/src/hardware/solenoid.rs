use vexide::{
    devices::{adi::digital::LogicLevel, PortError},
    prelude::AdiDigitalOut,
};

pub trait Solenoid {
    type Error;

    fn set_level(&mut self, level: LogicLevel) -> Result<(), Self::Error>;
    fn level(&self) -> Result<LogicLevel, Self::Error>;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.level()?.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.level()?.is_high())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(LogicLevel::High)
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(LogicLevel::Low)
    }

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.set_level(!self.level()?)
    }
}

impl Solenoid for AdiDigitalOut {
    type Error = PortError;

    fn set_level(&mut self, level: LogicLevel) -> Result<(), Self::Error> {
        self.set_level(level)
    }

    fn level(&self) -> Result<LogicLevel, Self::Error> {
        self.level()
    }
}
