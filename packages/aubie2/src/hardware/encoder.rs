use evian::tracking::RotarySensor;
use vexide::{
    devices::PortError,
    prelude::{AdiEncoder, AdiPort, Direction, Position},
};

pub struct CustomEncoder<const TPR: u32> {
    enc: AdiEncoder,
    direction: Direction,
}

impl<const TPR: u32> CustomEncoder<TPR> {
    pub fn new(top_port: AdiPort, bottom_port: AdiPort, direction: Direction) -> Self {
        let enc = AdiEncoder::new(top_port, bottom_port);

        Self { enc, direction }
    }
}

impl<const TPR: u32> RotarySensor for CustomEncoder<TPR> {
    type Error = PortError;

    fn position(&self) -> Result<Position, Self::Error> {
        Ok(Position::from_ticks(
            self.enc
                .position()?
                .as_ticks(AdiEncoder::TICKS_PER_REVOLUTION)
                * match self.direction {
                    Direction::Forward => 1,
                    Direction::Reverse => -1,
                },
            TPR,
        ))
    }
}
