extern crate pyo3;
use pyo3::prelude::*;

pub mod utils;
pub mod objectives;
pub mod lively_tk;

use crate::utils::state::State;
use crate::utils::collision_object::{*};
use crate::utils::geometry::{*};
use crate::utils::robot_model::RobotModel;
use crate::objectives::core::base::{*};
use crate::objectives::core::bounding::{*};
use crate::objectives::core::matching::{*};
use crate::objectives::core::mirroring::{*};
use crate::objectives::liveliness::forces::{*};
use crate::objectives::liveliness::perlin::{*};
use crate::lively_tk::Solver;

#[pymodule]
fn lively_tk(_py: Python, m: &PyModule) -> PyResult<()> {
    // State
    m.add_class::<State>()?;
    // CollisionObjects
    m.add_class::<BoxObject>()?;
    m.add_class::<SphereObject>()?;
    m.add_class::<CylinderObject>()?;
    m.add_class::<CapsuleObject>()?;
    // Robot Model
    m.add_class::<RobotModel>()?;
    // Geometry
    m.add_class::<Translation>()?;
    m.add_class::<Rotation>()?;
    m.add_class::<Transform>()?;
    // Objectives
    m.add_class::<PositionMatchObjective>()?;
    m.add_class::<OrientationMatchObjective>()?;
    m.add_class::<PositionLivelinessObjective>()?;
    m.add_class::<OrientationLivelinessObjective>()?;
    m.add_class::<PositionMirroringObjective>()?;
    m.add_class::<OrientationMirroringObjective>()?;
    m.add_class::<PositionBoundingObjective>()?;
    m.add_class::<OrientationBoundingObjective>()?;
    m.add_class::<JointMatchObjective>()?;
    m.add_class::<JointLivelinessObjective>()?;
    m.add_class::<JointMirroringObjective>()?;
    m.add_class::<JointLimitsObjective>()?;
    m.add_class::<CollisionAvoidanceObjective>()?;
    m.add_class::<MinimizeVelocityObjective>()?;
    m.add_class::<MinimizeAccelerationObjective>()?;
    m.add_class::<MinimizeJerkObjective>()?;
    m.add_class::<RelativeMotionLivelinessObjective>()?;
    m.add_class::<RootPositionLivelinessObjective>()?;
    m.add_class::<RootPositionMatchObjective>()?;
    m.add_class::<GravityObjective>()?;
    m.add_class::<MacroSmoothness>()?;
    m.add_class::<DistanceMatchObjective>()?;
    // Solver
    m.add_class::<Solver>()?;
    Ok(())
}
