#[cfg(feature = "pybindings")]
extern crate pyo3;
#[cfg(feature = "pybindings")]
extern crate pyo3_log;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "jsbindings")]
use wasm_bindgen::prelude::*;
#[cfg(feature = "jsbindings")]
extern crate console_error_panic_hook;

pub mod utils;
pub mod objectives;
pub mod lively;
// pub mod manager;

#[cfg(feature = "pybindings")]
use crate::utils::pyutils::*;
#[cfg(feature = "pybindings")]
use crate::utils::state::State;
#[cfg(feature = "pybindings")]
use crate::utils::info::*;
#[cfg(feature = "pybindings")]
use crate::utils::shapes::*;

// Export struct from lively
pub use self::lively::Solver;

#[cfg(feature = "pybindings")]
#[pymodule]
#[pyo3(name = "lively")]
fn lively(_py: Python, m: &PyModule) -> PyResult<()> {
    pyo3_log::init();
    // State
    m.add_class::<State>()?;
    // Info
    m.add_class::<MimicInfo>()?;
    m.add_class::<JointInfo>()?;
    m.add_class::<LinkInfo>()?;
    m.add_class::<ProximityInfo>()?;
    m.add_class::<TransformInfo>()?;
    m.add_class::<CollisionSettingInfo>()?;
    // Shapes
    m.add_class::<BoxShape>()?;
    m.add_class::<SphereShape>()?;
    m.add_class::<CylinderShape>()?;
    m.add_class::<CapsuleShape>()?;
    m.add_class::<HullShape>()?;
    m.add_class::<MeshShape>()?;
    // Geometry/Goals
    m.add_class::<PySize>()?;
    m.add_class::<PyTranslation>()?;
    m.add_class::<PyRotation>()?;
    m.add_class::<PyTransform>()?;
    m.add_class::<Ellipse>()?;
    m.add_class::<RotationRange>()?;
    m.add_class::<ScalarRange>()?;
    m.add_class::<Line>()?;
    // ShapeUpdates
    m.add_class::<AddShape>()?;
    m.add_class::<MoveShape>()?;
    // Objectives
    m.add_class::<objectives::core::base::JointLimitsObjective>()?;
    m.add_class::<objectives::core::base::CollisionAvoidanceObjective>()?;
    m.add_class::<objectives::core::base::JointVelocityMinimizationObjective>()?;
    m.add_class::<objectives::core::base::JointAccelerationMinimizationObjective>()?;
    m.add_class::<objectives::core::base::JointJerkMinimizationObjective>()?;
    m.add_class::<objectives::core::base::OriginVelocityMinimizationObjective>()?;
    m.add_class::<objectives::core::base::OriginAccelerationMinimizationObjective>()?;
    m.add_class::<objectives::core::base::OriginJerkMinimizationObjective>()?;
    m.add_class::<objectives::core::base::SmoothnessMacroObjective>()?;
    m.add_class::<objectives::core::bounding::PositionBoundingObjective>()?;
    m.add_class::<objectives::core::bounding::OrientationBoundingObjective>()?;
    m.add_class::<objectives::core::bounding::JointBoundingObjective>()?;
    m.add_class::<objectives::core::matching::PositionMatchObjective>()?;
    m.add_class::<objectives::core::matching::OrientationMatchObjective>()?;
    m.add_class::<objectives::core::matching::JointMatchObjective>()?;
    m.add_class::<objectives::core::matching::DistanceMatchObjective>()?;
    m.add_class::<objectives::core::matching::PositionLineMatchObjective>()?;
    m.add_class::<objectives::core::mirroring::PositionMirroringObjective>()?;
    m.add_class::<objectives::core::mirroring::OrientationMirroringObjective>()?;
    m.add_class::<objectives::core::mirroring::JointMirroringObjective>()?;
    m.add_class::<objectives::liveliness::forces::GravityObjective>()?;
    m.add_class::<objectives::liveliness::perlin::PositionLivelinessObjective>()?;
    m.add_class::<objectives::liveliness::perlin::OrientationLivelinessObjective>()?;
    m.add_class::<objectives::liveliness::perlin::JointLivelinessObjective>()?;
    m.add_class::<objectives::liveliness::perlin::RelativeMotionLivelinessObjective>()?;
    // Solver
    m.add_class::<lively::Solver>()?;
    Ok(())
}

#[cfg(feature = "jsbindings")]
#[wasm_bindgen]
extern "C" {
    // Use `js_namespace` here to bind `console.log(..)` instead of just
    // `log(..)`
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}