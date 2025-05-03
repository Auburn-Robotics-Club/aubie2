use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use evian::{
    math::{Angle, Vec2},
    prelude::{
        TrackingWheel, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
        WheeledTracking,
    },
    tracking::RotarySensor,
};
use vexide::{
    devices::smart::InertialSensor,
    task::{spawn, Task},
    time::sleep,
};

use crate::hardware::Otos;

pub struct OtosTracking {
    // wheeled: WheeledTracking,
    // position: Rc<RefCell<Option<Vec2<f64>>>>,
    // _task: Task<()>,
}

impl OtosTracking {
    pub async fn new<const NUM_FORWARD: usize, T: RotarySensor + 'static>(
        origin: impl Into<Vec2<f64>>,
        heading: Angle,
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        mut otos: Otos,
        imu: InertialSensor,
    ) -> Self {
        // let position = Rc::new(RefCell::new(otos.position().await.ok().map(|pos| Vec2 {
        //     x: pos.x as f64,
        //     y: pos.y as f64,
        // })));

    //     Self {
    //         position: position.clone(),
    //         wheeled: WheeledTracking::forward_only(origin, heading, forward_wheels, Some(imu)),
    //         _task: spawn(async move {
    //             if position.borrow().is_none() {
    //                 return;
    //             }

    //             loop {
    //                 let Ok(pos) = otos.position().await else {
    //                     return;
    //                 };

    //                 *position.borrow_mut() = Some(Vec2 {
    //                     x: pos.x as f64,
    //                     y: pos.y as f64,
    //                 });

    //                 sleep(Duration::from_millis(5)).await;
    //             }
    //         }),
    //     }
    // }

    Self{}
    }
}

// impl TracksPosition for OtosTracking {
//     fn position(&self) -> Vec2<f64> {
//         match *self.position.borrow() {
//             Some(position) => position,
//             None => self.wheeled.position(),
//         }
//     }
// }

// impl TracksHeading for OtosTracking {
//     fn heading(&self) -> Angle {
//         self.wheeled.heading()
//     }
// }

// impl TracksForwardTravel for OtosTracking {
//     fn forward_travel(&self) -> f64 {
//         self.wheeled.forward_travel()
//     }
// }

// impl TracksVelocity for OtosTracking {
//     fn angular_velocity(&self) -> f64 {
//         self.wheeled.angular_velocity()
//     }

//     fn linear_velocity(&self) -> f64 {
//         self.wheeled.linear_velocity()
//     }
// }
