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

// #[cfg(feature = "jsbindings")]
// #[wasm_bindgen(js_name=Solver)]
// pub struct JsSolver(Solver);

// #[cfg(feature = "jsbindings")]
// impl Solver {
//     #[wasm_bindgen(constructor)]
//     pub fn new(
//         urdf: String, 
//         objectives: JsValue, 
//         root_bounds: JsValue,
//         shapes: JsValue,
//         initial_state: JsValue,
//         max_retries: Option<usize>,
//         max_iterations: Option<usize>,
//         collision_settings: JsValue
//     ) -> Self {
//             console_error_panic_hook::set_once();
//             let inner_objectives:HashMap<String,Objective> = serde_wasm_bindgen::from_value(objectives).unwrap();
//             let inner_bounds:Option<Vec<ScalarRange>> = serde_wasm_bindgen::from_value(root_bounds).unwrap();
//             let inner_shapes:Option<Vec<Shape>> = serde_wasm_bindgen::from_value(shapes).unwrap();
//             let inner_state:Option<State> = serde_wasm_bindgen::from_value(initial_state).unwrap();
//             let inner_collision_settings:Option<CollisionSettingInfo> = serde_wasm_bindgen::from_value(collision_settings).unwrap();
//             Self(Solver::new(urdf, inner_objectives, inner_bounds, inner_shapes, inner_state, max_retries, max_iterations, inner_collision_settings))
//     }

//     #[wasm_bindgen(getter)]
//     pub fn objectives(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
//         return serialize(&self.0.objective_set.objectives)
//     }

//     #[wasm_bindgen(setter)]
//     pub fn set_objectives(&mut self, objectives: JsValue) {
//         let inner_objectives: HashMap<String,Objective> = serde_wasm_bindgen::from_value(objectives).unwrap();
//         self.0.set_objectives(inner_objectives);
//     }

//     #[wasm_bindgen(getter = currentState)]
//     pub fn current_state(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
//         return serialize(&self.0.get_current_state())
//     }

//     #[wasm_bindgen(getter = currentGoals)]
//     pub fn current_goals(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
//         return serialize(&self.0.get_goals())
//     }

//     #[wasm_bindgen(getter)]
//     pub fn links(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
//         return serialize(self.0.get_links())
//     }

//     #[wasm_bindgen(getter)]
//     pub fn joints(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
//         return serialize(self.0.get_joints())
//     }

//     pub fn reset(
//         &mut self, 
//         state: JsValue,
//         weights: JsValue,
//     ) {
//         let inner_state:State = serde_wasm_bindgen::from_value(state).unwrap();
//         let inner_weights:HashMap<String,f64> = serde_wasm_bindgen::from_value(weights).unwrap();
//         self.0.reset(inner_state,inner_weights);
//     }

//     pub fn solve(
//         &mut self,
//         goals: JsValue,
//         weights: JsValue,
//         time: f64,
//         shape_updates: JsValue
//     ) -> Result<JsValue,serde_wasm_bindgen::Error> {
//         let inner_goals: HashMap<String,Goal> = serde_wasm_bindgen::from_value(goals).unwrap();
//         let inner_weights:HashMap<String,f64> = serde_wasm_bindgen::from_value(weights).unwrap();
//         let inner_updates: Option<Vec<ShapeUpdate>> = serde_wasm_bindgen::from_value(shape_updates).unwrap();
//         let state:State = self.0.solve(inner_goals,inner_weights,time,inner_updates);
//         return serialize(&state);
//     }

//     #[wasm_bindgen(js_name = computeAverageDistanceTable)]
//     pub fn compute_average_distance_table(&mut self) -> Result<JsValue,serde_wasm_bindgen::Error> {
//         return serialize(&self.0.compute_average_distance_table())
//     }
// }

#[cfg(feature = "jsbindings")]
#[wasm_bindgen]
extern "C" {
    // Use `js_namespace` here to bind `console.log(..)` instead of just
    // `log(..)`
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}