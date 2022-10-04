#[cfg(feature = "pybindings")]
extern crate pyo3;
#[cfg(feature = "pybindings")]
extern crate pyo3_log;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "jsbindings")]
use serde::{Serialize,Deserialize};
#[cfg(feature = "jsbindings")]
use wasm_bindgen::prelude::*;
#[cfg(feature = "jsbindings")]
extern crate console_error_panic_hook;

pub mod utils;
pub mod objectives;
pub mod lively;
pub mod wrappers;

#[cfg(feature = "pybindings")]
use crate::wrappers::python::solver::PySolver;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::geometry::{*};
#[cfg(feature = "pybindings")]
use crate::wrappers::python::objectives::{*};
#[cfg(feature = "pybindings")]
use crate::wrappers::python::shapes::{*};
#[cfg(feature = "pybindings")]
use crate::wrappers::python::state::PyState;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::info::{*};

#[cfg(feature = "jsbindings")]
use std::collections::HashMap;
#[cfg(feature = "jsbindings")]
use crate::utils::state::State;
#[cfg(feature = "jsbindings")]
use crate::utils::goals::*;
#[cfg(feature = "jsbindings")]
use crate::utils::info::*;
#[cfg(feature = "jsbindings")]
use crate::utils::shapes::*;
#[cfg(feature = "jsbindings")]
use crate::objectives::objective::*;
#[cfg(feature = "jsbindings")]
use crate::lively::Solver;

#[cfg(feature = "pybindings")]
#[pymodule]
fn lively_tk(_py: Python, m: &PyModule) -> PyResult<()> {
    pyo3_log::init();
    // State
    m.add_class::<PyState>()?;
    // Info
    m.add_class::<PyMimicInfo>()?;
    m.add_class::<PyJointInfo>()?;
    m.add_class::<PyLinkInfo>()?;
    m.add_class::<PyProximityInfo>()?;
    m.add_class::<PyTransformInfo>()?;
    m.add_class::<PyCollisionSettingInfo>()?;
    // Shapes
    m.add_class::<PyBoxShape>()?;
    m.add_class::<PySphereShape>()?;
    m.add_class::<PyCylinderShape>()?;
    m.add_class::<PyCapsuleShape>()?;
    m.add_class::<PyHullShape>()?;
    m.add_class::<PyMeshShape>()?;
    // m.add_class::<PyBoxZone>()?;
    // m.add_class::<PySphereZone>()?;
    // m.add_class::<PyCylinderZone>()?;
    // m.add_class::<PyCapsuleZone>()?;
    // Geometry/Goals
    m.add_class::<Size>()?;
    m.add_class::<Translation>()?;
    m.add_class::<Rotation>()?;
    m.add_class::<Transform>()?;
    m.add_class::<Ellipse>()?;
    m.add_class::<RotationRange>()?;
    m.add_class::<ScalarRange>()?;
    // Objectives
    m.add_class::<PyPositionMatchObjective>()?;
    m.add_class::<PyOrientationMatchObjective>()?;
    m.add_class::<PyPositionLivelinessObjective>()?;
    m.add_class::<PyOrientationLivelinessObjective>()?;
    m.add_class::<PyPositionMirroringObjective>()?;
    m.add_class::<PyOrientationMirroringObjective>()?;
    m.add_class::<PyPositionBoundingObjective>()?;
    m.add_class::<PyOrientationBoundingObjective>()?;
    m.add_class::<PyJointBoundingObjective>()?;
    m.add_class::<PyJointMatchObjective>()?;
    m.add_class::<PyJointLivelinessObjective>()?;
    m.add_class::<PyJointMirroringObjective>()?;
    m.add_class::<PyJointLimitsObjective>()?;
    m.add_class::<PyCollisionAvoidanceObjective>()?;
    m.add_class::<PyVelocityMinimizationObjective>()?;
    m.add_class::<PyAccelerationMinimizationObjective>()?;
    m.add_class::<PyJerkMinimizationObjective>()?;
    m.add_class::<PyOriginVelocityMinimizationObjective>()?;
    m.add_class::<PyOriginAccelerationMinimizationObjective>()?;
    m.add_class::<PyOriginJerkMinimizationObjective>()?;
    m.add_class::<PyRelativeMotionLivelinessObjective>()?;
    //m.add_class::<PyOriginPositionLivelinessObjective>()?;
    //m.add_class::<PyOriginPositionMatchObjective>()?;
    m.add_class::<PyGravityObjective>()?;
    m.add_class::<PySmoothnessMacroObjective>()?;
    m.add_class::<PyDistanceMatchObjective>()?;
    // Solver
    m.add_class::<PySolver>()?;
    Ok(())
}

#[cfg(feature = "jsbindings")]
#[derive(Serialize,Deserialize,Clone,Debug)]
pub struct ScalarRange {
    value: f64,
    delta: f64
}

#[cfg(feature = "jsbindings")]
#[wasm_bindgen(js_name=Solver)]
pub struct JsSolver(Solver);

#[cfg(feature = "jsbindings")]
#[wasm_bindgen(js_class=Solver)]
impl JsSolver {
    #[wasm_bindgen(constructor)]
    pub fn new(
        urdf: String, 
        objectives: &JsValue, 
        root_bounds: &JsValue,
        shapes: &JsValue,
        initial_state: &JsValue,
        max_retries: Option<usize>,
        max_iterations: Option<usize>,
        collision_settings: &JsValue
    ) -> Self {
            console_error_panic_hook::set_once();
            let inner_objectives:HashMap<String,Objective> = objectives.into_serde().unwrap();
            let temp_bounds:Option<Vec<ScalarRange>> = root_bounds.into_serde().unwrap();
            let inner_bounds:Option<Vec<(f64,f64)>> = temp_bounds.map(|bs| bs.iter().map(|b| (b.value,b.delta)).collect());
            let inner_shapes:Option<Vec<Shape>> = shapes.into_serde().unwrap();
            let inner_state:Option<State> = initial_state.into_serde().unwrap();
            let inner_collision_settings:Option<CollisionSettingInfo> = collision_settings.into_serde().unwrap();
            // let inner_retries: Option<u64> = max_retries.into_serde().unwrap();
            // let inner_iterations: Option<usize> = max_iterations.into_serde().unwrap();
            // let inner_core: Option<bool> = only_core.into_serde().unwrap();
            Self(Solver::new(urdf, inner_objectives, inner_bounds, inner_shapes, inner_state, max_retries, max_iterations, inner_collision_settings))
    }

    #[wasm_bindgen(getter)]
    pub fn objectives(&self) -> JsValue {
        JsValue::from_serde(&self.0.objective_set.objectives).unwrap()
    }

    #[wasm_bindgen(setter)]
    pub fn set_objectives(&mut self, objectives: JsValue) {
        let inner_objectives: HashMap<String,Objective> = objectives.into_serde().unwrap();
        self.0.set_objectives(inner_objectives);
    }

    #[wasm_bindgen(getter = currentState)]
    pub fn current_state(&self) -> JsValue {
        JsValue::from_serde(&self.0.get_current_state()).unwrap()
    }

    #[wasm_bindgen(getter = currentGoals)]
    pub fn current_goals(&self) -> JsValue {
        let mut goals: HashMap<String,Option<Goal>> = HashMap::new();
        for (k,v) in self.0.objective_set.objectives.iter() {
            goals.insert(k.clone(),v.get_goal());
        }
        JsValue::from_serde(&goals).unwrap()
    }

    #[wasm_bindgen(getter)]
    pub fn links(&self) -> JsValue {
        JsValue::from_serde(&self.0.robot_model.links).unwrap()
    }

    #[wasm_bindgen(getter)]
    pub fn joints(&self) -> JsValue {
        JsValue::from_serde(&self.0.robot_model.joints).unwrap()
    }

    pub fn reset(
        &mut self, 
        state: &JsValue,
        weights: &JsValue,
    ) {
        let inner_state:State = state.into_serde().unwrap();
        let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
        self.0.reset(inner_state,inner_weights);
    }

    pub fn solve(
        &mut self,
        goals: &JsValue,
        weights: &JsValue,
        time: f64,
        shape_updates: &JsValue
    ) -> JsValue {
        let inner_goals: HashMap<String,Goal> = goals.into_serde().unwrap();
        let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
        let inner_updates: Option<Vec<ShapeUpdate>> = shape_updates.into_serde().unwrap();
        let state:State = self.0.solve(inner_goals,inner_weights,time,inner_updates);
        return JsValue::from_serde(&state).unwrap();
    }

    #[wasm_bindgen(js_name = computeAverageDistanceTable)]
    pub fn compute_average_distance_table(&mut self) -> JsValue {
        return JsValue::from_serde(&self.0.compute_average_distance_table()).unwrap()
    }
}

#[cfg(feature = "jsbindings")]
#[wasm_bindgen]
extern "C" {
    // Use `js_namespace` here to bind `console.log(..)` instead of just
    // `log(..)`
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

// #[cfg(feature = "jsbindings")]
// macro_rules! console_log {
//     // Note that this is using the `log` function imported above during
//     // `bare_bones`
//     ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
// }

#[cfg(feature = "jsbindings")]
#[wasm_bindgen]
pub fn solve(solver: &mut JsSolver, goals: &JsValue, weights: &JsValue, time: f64, shape_updates: &JsValue) -> JsValue {
    let inner_goals: HashMap<String,Goal> = goals.into_serde().unwrap();
    let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
    let inner_updates: Option<Vec<ShapeUpdate>> = shape_updates.into_serde().unwrap();
    // console_log!("Received Goals: {:?}",inner_goals);
    // console_log!("Received Weights: {:?}",inner_weights);
    // console_log!("Received Updates: {:?}",inner_updates);
    let state:State = solver.0.solve(inner_goals,inner_weights,time,inner_updates);
    // console_log!("Produced State: {:?}",state);
    return JsValue::from_serde(&state).unwrap();
}

#[cfg(feature = "jsbindings")]
#[wasm_bindgen]
pub fn reset(solver: &mut JsSolver, state: &JsValue, weights: &JsValue) {
    let inner_state:State = state.into_serde().unwrap();
    let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
    solver.0.reset(inner_state,inner_weights);
}