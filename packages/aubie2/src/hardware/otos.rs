//! AUBIE Solenoid SwitchBoard

use alloc::vec::Vec;
use core::time::Duration;

use evian::math::{Angle, Vec2};
use vexide::{
    devices::smart::{
        serial::{SerialError, SerialPort},
        SmartPort,
    },
    io::{self, println, Read, Write},
    time::sleep,
};

/// SparkFun Optical Tracking Odometry Sensor (OTOS)
#[derive(Debug)]
#[non_exhaustive]
pub struct Otos {
    pub serial: SerialPort,
}

impl Otos {
    pub async fn new(port: SmartPort) -> Self {
        let serial = SerialPort::open(port, 9600).await;

        Self { serial }
    }

    fn cmd(&mut self, cmd: Command) -> Result<(), io::Error> {
        let mut buf = Vec::with_capacity(16);

        buf.push(cmd.opcode());

        match cmd {
            Command::SetSolenoids(state) => {
                buf.push(state);
            }
            Command::SetOffset(position, heading) => {
                buf.extend_from_slice(&position.x.to_le_bytes());
                buf.extend_from_slice(&position.y.to_le_bytes());
                buf.extend_from_slice(&(heading.as_degrees() as f32).to_le_bytes());
            }
            Command::SetPose(position, heading) => {
                buf.extend_from_slice(&position.x.to_le_bytes());
                buf.extend_from_slice(&position.y.to_le_bytes());
                buf.extend_from_slice(&(heading.as_degrees() as f32).to_le_bytes());
            }
            Command::SetScalar(scalar) => buf.extend_from_slice(&scalar.to_le_bytes()),
            _ => {}
        }

        self.serial.write_all(&buf)
    }

    pub async fn calibrate(&mut self) -> Result<u8, io::Error> {
        self.cmd(Command::Calibrate)?;

        loop {
            if self.serial.unread_bytes().unwrap() != 0 {
                println!("{:x?}", self.serial.unread_bytes());
            }

            sleep(Duration::from_millis(5)).await;
        }

        while self.serial.unread_bytes().map_err(|err| match err {
            SerialError::ReadFailed => {
                io::Error::new(io::ErrorKind::Other, "unread_bytes failed unexpectedly")
            }
            _ => unreachable!(),
        })? == 0
        {
            sleep(Duration::from_millis(1)).await;
        }

        self.serial.read_byte().unwrap();

        Ok(self.serial.read_byte().unwrap())
    }

    pub fn set_scalar(&mut self, scalar: f32) -> Result<(), io::Error> {
        self.cmd(Command::SetScalar(scalar))
    }

    pub fn set_offset(
        &mut self,
        position_offset: Vec2<f32>,
        heading_offset: Angle,
    ) -> Result<(), io::Error> {
        self.cmd(Command::SetOffset(position_offset, heading_offset))
    }

    pub fn set_pose(
        &mut self,
        position_offset: Vec2<f32>,
        heading_offset: Angle,
    ) -> Result<(), io::Error> {
        self.cmd(Command::SetPose(position_offset, heading_offset))
    }

    pub async fn position(&mut self) -> Result<Vec2<f32>, io::Error> {
        self.cmd(Command::Position)?;

        const PAYLOAD_SIZE: usize = size_of::<f32>() * 2;

        while self.serial.unread_bytes().map_err(|err| match err {
            SerialError::ReadFailed => {
                io::Error::new(io::ErrorKind::Other, "unread_bytes failed unexpectedly")
            }
            _ => unreachable!(),
        })? < (size_of::<f32>() * 2)
        {
            sleep(Duration::from_millis(1)).await;
        }

        let mut buf = [0; PAYLOAD_SIZE];
        self.serial.read(&mut buf).unwrap();

        Ok(Vec2 {
            x: f32::from_le_bytes(buf[0..4].try_into().unwrap()),
            y: f32::from_le_bytes(buf[4..8].try_into().unwrap()),
        })
    }
}

#[repr(u8)]
pub(crate) enum Command {
    #[allow(unused)]
    SetSolenoids(u8),
    SetScalar(f32),
    SetOffset(Vec2<f32>, Angle),
    SetPose(Vec2<f32>, Angle),
    Calibrate,
    Position,
}

impl Command {
    const fn opcode(&self) -> u8 {
        match self {
            Self::SetSolenoids(_) => 0b01010111,
            Self::SetScalar(_) => 0b11011000,
            Self::SetOffset(_, _) => 0b11011001,
            Self::SetPose(_, _) => 0b11011100,
            Self::Calibrate => 0b11011010,
            Self::Position => 0b11011011,
        }
    }
}
