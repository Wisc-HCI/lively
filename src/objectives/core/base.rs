use crate::objectives::objective::{groove_loss, Callable};
use crate::utils::state::State;
use crate::utils::vars::Vars;
use nalgebra::Vector3;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
use serde::{Deserialize, Serialize};

const Y_OFFSET: f64 = 0.8808; //1.0 / (1.0 as f64 + (-2.0 as f64).exp());
const TEMPORAL_NORMALIZATION: f64 = 30.0; // Assume normally it runs around 30 frames per second

/*
Collision Avoidance
*/
#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct CollisionAvoidanceObjective {
    pub name: String,
    pub weight: f64,
}

impl CollisionAvoidanceObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for CollisionAvoidanceObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        let mut score: f64 = 0.0;

        for proximity_info in &state.proximity {
            if proximity_info.physical {
                let score_offset = 1.0
                    / (1.0 + (-200.0 * proximity_info.average_distance.unwrap_or(1.0) + 8.0).exp());
                if proximity_info.distance < 0.0 {
                    score += (-10.0 * proximity_info.distance + Y_OFFSET) * score_offset;
                } else {
                    score += (1.0 / (1.0 + (100.0 * (proximity_info.distance / 1.0) - 2.0).exp()))
                        * score_offset;
                }
                score += proximity_info.distance
            }
        }

        return self.weight * groove_loss(score, 0.0, 2, 0.32950, 0.1, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl CollisionAvoidanceObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        CollisionAvoidanceObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

/*
Joint Limits
*/
#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct JointLimitsObjective {
    pub name: String,
    pub weight: f64,
}

impl JointLimitsObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for JointLimitsObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a: f64 = 0.05 / (penalty_cutoff.powi(50));
        for joint in v.joints.iter() {
            let l: f64 = joint.lower_bound;
            let u: f64 = joint.upper_bound;
            let joint_value = state.get_joint_position(&joint.name);
            if u - l <= 0.0 {
                // In cases where the upper and lower limits are the same,
                // just compare the lower limit to the x value.
                x_val += a * (joint_value - l).abs().powi(2);
            } else {
                // Otherwise, compare as normal
                let r: f64 = (joint_value - l) / (u - l);
                let n: f64 = 2.0 * (r - 0.5).abs();
                x_val += a * n.powi(50);
            }
        }
        // println!("JointLimits error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JointLimitsObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        JointLimitsObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

/*
# Velocity-Based Smoothness
*/
#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct JointVelocityMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl JointVelocityMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for JointVelocityMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val = 0.0;
        for joint in v.joints.iter() {
            let time_diff:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
            let joint_value: f64 = state.get_joint_position(&joint.name);
            if time_diff > 0.0 {
                x_val += ((joint_value - v.history.prev1.get_joint_position(&joint.name))
                    / time_diff)
                    .powi(2);
            } else {
                x_val += (joint_value - v.history.prev1.get_joint_position(&joint.name)).powi(2);
            }
        }
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JointVelocityMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        JointVelocityMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct OriginVelocityMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl OriginVelocityMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for OriginVelocityMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let past: Vector3<f64> = v.history.prev1.origin.translation.vector;
        let time_diff:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
        let x_val: f64;
        if time_diff > 0.0 {
            x_val = ((state.origin.translation.vector - past).norm() / time_diff).powi(2);
        } else {
            x_val = (state.origin.translation.vector - past).norm().powi(2);
        }
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OriginVelocityMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        OriginVelocityMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct LinkVelocityMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl LinkVelocityMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for LinkVelocityMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val: f64 = 0.0;
        for link in v.links.iter() {
            let past: Vector3<f64> = v.history.prev1.get_link_transform(&link.name).translation.vector;
            let current: Vector3<f64> = state.get_link_transform(&link.name).translation.vector;
            let time_diff:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
            if time_diff > 0.0 {
                x_val += ((current - past).norm() / time_diff).powi(2);
            } else {
                x_val += (current - past).norm().powi(2);
            }
        }
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl LinkVelocityMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        LinkVelocityMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct JointAccelerationMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl JointAccelerationMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for JointAccelerationMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val = 0.0;
        let time_diff1:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
        let time_diff2:f64 = TEMPORAL_NORMALIZATION*(v.history.prev1.timestamp - v.history.prev2.timestamp);
        for joint in v.joints.iter() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            let v1: f64 = joint_value - v.history.prev1.get_joint_position(&joint.name);
            let v2: f64 = v.history.prev1.get_joint_position(&joint.name)
                - v.history.prev2.get_joint_position(&joint.name);
            if time_diff1 > 0.0 && time_diff2 > 0.0 {
                x_val += (v1 / time_diff1 - v2 / time_diff2).powi(2);
            } else {
                x_val += (v1 - v2).powi(2);
            }
        }
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JointAccelerationMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        JointAccelerationMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct OriginAccelerationMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl OriginAccelerationMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for OriginAccelerationMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let pos1: Vector3<f64> = state.origin.translation.vector;
        let pos2: Vector3<f64> = v.history.prev1.origin.translation.vector;
        let pos3: Vector3<f64> = v.history.prev2.origin.translation.vector;
        let time_diff1:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
        let time_diff2:f64 = TEMPORAL_NORMALIZATION*(v.history.prev1.timestamp - v.history.prev2.timestamp);
        let x_val: f64;
        if time_diff1 > 0.0 && time_diff2 > 0.0 {
            x_val = ((pos1 - pos2)/time_diff1 - (pos2 - pos3)/time_diff2).norm().powi(2);
        } else {
            x_val = ((pos1 - pos2) - (pos2 - pos3)).norm().powi(2);
        }

        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OriginAccelerationMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        OriginAccelerationMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct LinkAccelerationMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl LinkAccelerationMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for LinkAccelerationMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val: f64 = 0.0;
        let time_diff1:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
        let time_diff2:f64 = TEMPORAL_NORMALIZATION*(v.history.prev1.timestamp - v.history.prev2.timestamp);
        for link in v.links.iter() {
            let pos1: Vector3<f64> = state.get_link_transform(&link.name).translation.vector;
            let pos2: Vector3<f64> = v.history.prev1.get_link_transform(&link.name).translation.vector;
            let pos3: Vector3<f64> = v.history.prev2.get_link_transform(&link.name).translation.vector;
            
            if time_diff1 > 0.0 && time_diff2 > 0.0 {
                x_val += ((pos1 - pos2)/time_diff1 - (pos2 - pos3)/time_diff2).norm().powi(2);
            } else {
                x_val += ((pos1 - pos2) - (pos2 - pos3)).norm().powi(2);
            }
        }
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl LinkAccelerationMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        LinkAccelerationMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct JointJerkMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl JointJerkMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for JointJerkMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val = 0.0;
        let time_diff1:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
        let time_diff2:f64 = TEMPORAL_NORMALIZATION*(v.history.prev1.timestamp - v.history.prev2.timestamp);
        let time_diff3:f64 = TEMPORAL_NORMALIZATION*(v.history.prev2.timestamp - v.history.prev3.timestamp);
        for joint in v.joints.iter() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            let v1: f64;
            let v2: f64;
            let v3: f64;
            v1 = joint_value - v.history.prev1.get_joint_position(&joint.name);
            v2 = v.history.prev1.get_joint_position(&joint.name)
                - v.history.prev2.get_joint_position(&joint.name);
            v3 = v.history.prev2.get_joint_position(&joint.name)
                - v.history.prev3.get_joint_position(&joint.name);
            if time_diff1 > 0.0 && time_diff2 > 0.0 && time_diff3 > 0.0 {
                x_val += ((v1 / time_diff1 - v2 / time_diff2)
                    - (v2 / time_diff2 - v3 / time_diff3))
                    .powi(2);
            } else {
                x_val += ((v1 - v2) - (v2 - v3)).powi(2);
            }
        }
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JointJerkMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        JointJerkMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct OriginJerkMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl OriginJerkMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for OriginJerkMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let x_val: f64;
        let pos1: Vector3<f64> = state.origin.translation.vector;
        let pos2: Vector3<f64> = v.history.prev1.origin.translation.vector;
        let pos3: Vector3<f64> = v.history.prev2.origin.translation.vector;
        let pos4: Vector3<f64> = v.history.prev3.origin.translation.vector;
        let time_diff1:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
        let time_diff2:f64 = TEMPORAL_NORMALIZATION*(v.history.prev1.timestamp - v.history.prev2.timestamp);
        let time_diff3:f64 = TEMPORAL_NORMALIZATION*(v.history.prev2.timestamp - v.history.prev3.timestamp);
        if time_diff1 > 0.0 && time_diff2 > 0.0 && time_diff3 > 0.0 {
            x_val = (((pos1 - pos2)/time_diff1 - (pos2 - pos3)/time_diff2) - ((pos2 - pos3)/time_diff2 - (pos3 - pos4)/time_diff3))
            .norm()
            .powi(2);
        } else {
            x_val = (((pos1 - pos2) - (pos2 - pos3)) - ((pos2 - pos3) - (pos3 - pos4)))
            .norm()
            .powi(2);
        }
        
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OriginJerkMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        OriginJerkMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct LinkJerkMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl LinkJerkMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for LinkJerkMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val: f64 = 0.0;
        let time_diff1:f64 = TEMPORAL_NORMALIZATION*(state.timestamp - v.history.prev1.timestamp);
        let time_diff2:f64 = TEMPORAL_NORMALIZATION*(v.history.prev1.timestamp - v.history.prev2.timestamp);
        let time_diff3:f64 = TEMPORAL_NORMALIZATION*(v.history.prev2.timestamp - v.history.prev3.timestamp);
        for link in v.links.iter() {
            let pos1: Vector3<f64> = state.get_link_transform(&link.name).translation.vector;
            let pos2: Vector3<f64> = v.history.prev1.get_link_transform(&link.name).translation.vector;
            let pos3: Vector3<f64> = v.history.prev2.get_link_transform(&link.name).translation.vector;
            let pos4: Vector3<f64> = v.history.prev3.get_link_transform(&link.name).translation.vector;
            
            if time_diff1 > 0.0 && time_diff2 > 0.0 && time_diff3 > 0.0 {
                x_val += (((pos1 - pos2)/time_diff1 - (pos2 - pos3)/time_diff2) - ((pos2 - pos3)/time_diff2 - (pos3 - pos4)/time_diff3))
                .norm()
                .powi(2);
            } else {
                x_val += (((pos1 - pos2) - (pos2 - pos3)) - ((pos2 - pos3) - (pos3 - pos4)))
                .norm()
                .powi(2);
            }
        }
        return self.weight * groove_loss(x_val.sqrt(), 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl LinkJerkMinimizationObjective {
    #[new]
    pub fn from_python(name: String, weight: f64) -> Self {
        LinkJerkMinimizationObjective::new(name, weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
#[cfg_attr(feature = "pybindings", pyclass)]
#[serde(from="SmoothnessMacroWrapper")]
// A macro wrapping the velocity/acceleration/jerk minimization objectives with sensible defaults.
pub struct SmoothnessMacroObjective {
    pub name: String,
    pub weight: f64,
    pub joints: bool,
    pub origin: bool,
    pub links: bool,
    joint_velocity_objective: Option<JointVelocityMinimizationObjective>,
    joint_acceleration_objective: Option<JointAccelerationMinimizationObjective>,
    joint_jerk_objective: Option<JointJerkMinimizationObjective>,
    origin_velocity_objective: Option<OriginVelocityMinimizationObjective>,
    origin_acceleration_objective: Option<OriginAccelerationMinimizationObjective>,
    origin_jerk_objective: Option<OriginJerkMinimizationObjective>,
    link_velocity_objective: Option<LinkVelocityMinimizationObjective>,
    link_acceleration_objective: Option<LinkAccelerationMinimizationObjective>,
    link_jerk_objective: Option<LinkJerkMinimizationObjective>,
}

impl SmoothnessMacroObjective {
    pub fn new(name: String, weight: f64, joints: bool, origin: bool, links: bool) -> Self {
        let joint_velocity_objective: Option<JointVelocityMinimizationObjective>;
        let joint_acceleration_objective: Option<JointAccelerationMinimizationObjective>;
        let joint_jerk_objective: Option<JointJerkMinimizationObjective>;

        if joints {
            joint_velocity_objective = Some(JointVelocityMinimizationObjective::new(
                format!("Macro {} Joint Velocity", name),
                0.21,
            ));
            joint_acceleration_objective = Some(JointAccelerationMinimizationObjective::new(
                format!("Macro {} Joint Accel", name),
                0.08,
            ));
            joint_jerk_objective = Some(JointJerkMinimizationObjective::new(
                format!("Macro {} Joint Jerk", name),
                0.04,
            ));
        } else {
            joint_velocity_objective = None;
            joint_acceleration_objective = None;
            joint_jerk_objective = None
        }

        let origin_velocity_objective: Option<OriginVelocityMinimizationObjective>;
        let origin_acceleration_objective: Option<OriginAccelerationMinimizationObjective>;
        let origin_jerk_objective: Option<OriginJerkMinimizationObjective>;

        if origin {
            origin_velocity_objective = Some(OriginVelocityMinimizationObjective::new(
                format!("Macro {} Origin Velocity", name),
                0.47
            ));
            origin_acceleration_objective = Some(OriginAccelerationMinimizationObjective::new(
                format!("Macro {} Origin Accel", name),
                0.15,
            ));
            origin_jerk_objective = Some(OriginJerkMinimizationObjective::new(
                format!("Macro {} Origin Jerk", name),
                0.05,
            ));
        } else {
            origin_velocity_objective = None;
            origin_acceleration_objective = None;
            origin_jerk_objective = None
        }

        let link_velocity_objective: Option<LinkVelocityMinimizationObjective>;
        let link_acceleration_objective: Option<LinkAccelerationMinimizationObjective>;
        let link_jerk_objective: Option<LinkJerkMinimizationObjective>;

        if links {
            link_velocity_objective = Some(LinkVelocityMinimizationObjective::new(
                format!("Macro {} Link Velocity", name),
                0.47
            ));
            link_acceleration_objective = Some(LinkAccelerationMinimizationObjective::new(
                format!("Macro {} Link Accel", name),
                0.15,
            ));
            link_jerk_objective = Some(LinkJerkMinimizationObjective::new(
                format!("Macro {} Link Jerk", name),
                0.05,
            ));
        } else {
            link_velocity_objective = None;
            link_acceleration_objective = None;
            link_jerk_objective = None;
        }

        Self {
            name: name.clone(),
            weight,
            joints, origin, links,
            joint_velocity_objective,
            joint_acceleration_objective,
            joint_jerk_objective,
            origin_velocity_objective,
            origin_acceleration_objective,
            origin_jerk_objective,
            link_velocity_objective,
            link_acceleration_objective,
            link_jerk_objective
        }
    }
}

impl Callable<bool> for SmoothnessMacroObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let velocity_cost = self.joint_velocity_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let acceleration_cost = self.joint_acceleration_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let jerk_cost = self.joint_jerk_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let origin_velocity_cost = self.origin_velocity_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let origin_acceleration_cost = self.origin_acceleration_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let origin_jerk_cost = self.origin_jerk_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let link_velocity_cost = self.link_velocity_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let link_acceleration_cost = self.link_acceleration_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        let link_jerk_cost = self.link_jerk_objective.as_ref().map_or_else(|| 0.0, |o| o.call(v,state));
        return self.weight
            * (velocity_cost
                + acceleration_cost
                + jerk_cost
                + origin_velocity_cost
                + origin_acceleration_cost
                + origin_jerk_cost
                + link_velocity_cost
                + link_acceleration_cost
                + link_jerk_cost
            );
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl SmoothnessMacroObjective {
    #[new]
    pub fn from_python(name: String, weight: f64, joints: bool, origin: bool, links: bool) -> Self {
        SmoothnessMacroObjective::new(name, weight, joints, origin, links)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
struct SmoothnessMacroWrapper {
    pub name: String,
    pub weight: f64,
    #[serde(default="get_true")]
    pub joints: bool,
    #[serde(default="get_false")]
    pub origin: bool,
    #[serde(default="get_true")]
    pub links: bool
}

impl From<SmoothnessMacroWrapper> for SmoothnessMacroObjective {
    fn from(wrapper: SmoothnessMacroWrapper) -> SmoothnessMacroObjective {
        SmoothnessMacroObjective::new(wrapper.name,wrapper.weight,wrapper.joints,wrapper.origin,wrapper.links)
    }
}

fn get_true() -> bool {true}
fn get_false() -> bool {false}