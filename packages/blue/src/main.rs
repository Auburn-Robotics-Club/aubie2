#![no_main]
#![no_std]

extern crate alloc;

use aubie2::{
    subsystems::{
        lady_brown::{LadyBrown, LadyBrownTarget},
        Intake,
    },
    theme::THEME_WAR_EAGLE,
};
use evian::{control::Pid, prelude::*};
use vexide::prelude::*;

const LADY_BROWN_LOWERED: Position = Position::from_degrees(289.0);
const LADY_BROWN_RAISED: Position = Position::from_degrees(327.0);

pub struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, ParallelWheelTracking>,
    intake: Intake<2>,
    lady_brown: LadyBrown<2, Pid>,
    lady_brown_state: LadyBrownTarget,
    clamp: AdiDigitalOut,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        // let dt = &mut self.drivetrain;
        // let mut mtp = MoveToPoint {
        //     distance_controller: Pid::new(0.5, 0.0, 0.0, None),
        //     angle_controller: AngularPid::new(0.5, 0.0, 0.0, None),
        //     settler: Settler::new()
        //         .error_tolerance(0.3)
        //         .tolerance_duration(Duration::from_millis(100))
        //         .timeout(Duration::from_secs(2)),
        // };
        println!("{:?}", self.drivetrain.tracking.heading().as_degrees());
        self.drivetrain.tracking.set_heading(Angle::ZERO);
        println!("{:?}", self.drivetrain.tracking.heading().as_degrees());

        // mtp.move_to_point(dt, (0.0, 10.0)).await;
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            // Single-stick arcade joystick control
            _ = self.drivetrain.motors.set_voltages(
                DifferentialVoltages::from_arcade(
                    state.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                    state.left_stick.x() * Motor::V5_MAX_VOLTAGE,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );

            // Raise/slower ladybrown when B is pressed.
            if state.button_l1.is_now_pressed() {
                self.lady_brown_state = match self.lady_brown_state {
                    LadyBrownTarget::Position(state) => match state {
                        LADY_BROWN_LOWERED => LadyBrownTarget::Position(LADY_BROWN_RAISED),
                        LADY_BROWN_RAISED => LadyBrownTarget::Position(LADY_BROWN_LOWERED),
                        _ => unreachable!(),
                    },
                    LadyBrownTarget::Manual(_) => LadyBrownTarget::Position(LADY_BROWN_RAISED),
                };
            }

            // Manual ladybrown control using R1/R2.
            if let LadyBrownTarget::Manual(_) = self.lady_brown_state {
                if state.right_stick.y() != 0.0 {
                    self.lady_brown_state = LadyBrownTarget::Manual(MotorControl::Voltage(
                        state.right_stick.y() * Motor::V5_MAX_VOLTAGE,
                    ));
                } else {
                    self.lady_brown_state =
                        LadyBrownTarget::Manual(MotorControl::Brake(BrakeMode::Hold));
                }
            }

            // Update ladybrown state machine with new possible target.
            _ = self.lady_brown.update(self.lady_brown_state);

            // Intake control - R1/R2.
            if state.button_b.is_pressed() {
                _ = self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if state.button_down.is_pressed() {
                _ = self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                _ = self.intake.set_voltage(0.0);
            }

            // A to toggle mogo mech.
            if state.button_a.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            sleep(Motor::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        // Controller
        controller: peripherals.primary_controller,

        // Drivetrain Model & Localization
        drivetrain: {
            // Left/right motors shared between drivetrain and odometry.
            let left_motors = shared_motors![
                Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
            ];
            let right_motors = shared_motors![
                Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_16, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
            ];

            // Drivetrain Model
            Drivetrain::new(
                Differential::new(left_motors.clone(), right_motors.clone()),
                ParallelWheelTracking::new(
                    Vec2::new(0.0, 0.0),
                    90.0.rad(),
                    TrackingWheel::new(left_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                    TrackingWheel::new(right_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                    None,
                ),
            )
        },

        // Intake
        intake: Intake::new([
            Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
        ]),

        // Lady Brown Arm
        lady_brown: LadyBrown::new(
            [
                Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
                Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward),
            ],
            RotationSensor::new(peripherals.port_4, Direction::Forward),
            Pid::new(0.45, 0.0, 0.001, None),
        ),
        lady_brown_state: LadyBrownTarget::Position(LADY_BROWN_LOWERED),

        // Mogo
        clamp: AdiDigitalOut::new(peripherals.adi_h),
    };

    robot.compete().await;
}
