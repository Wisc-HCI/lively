use crate::objectives::objective::{groove_loss, Callable};
use crate::utils::state::State;
use crate::utils::vars::Vars;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;

const Y_OFFSET: f64 = 0.8808; //1.0 / (1.0 as f64 + (-2.0 as f64).exp());

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

        return self.weight * score;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl CollisionAvoidanceObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        CollisionAvoidanceObjective::new(name,weight)
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
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a: f64 = 0.05 / (penalty_cutoff.powi(50));
        for joint in v.joints.iter() {
            let l: f64 = joint.lower_bound;
            let u: f64 = joint.upper_bound;
            let joint_value = state.get_joint_position(&joint.name);
            if u - l <= 0.0 {
                // In cases where the upper and lower limits are the same,
                // just compare the lower limit to the x value.
                sum += a * (joint_value - l).abs().powi(2);
            } else {
                // Otherwise, compare as normal
                let r: f64 = (joint_value - l) / (u - l);
                let n: f64 = 2.0 * (r - 0.5).abs();
                sum += a * n.powi(50);
            }
        }
        // println!("JointLimits error: {:?}",sum);
        return self.weight * groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JointLimitsObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        JointLimitsObjective::new(name,weight)
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
pub struct VelocityMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl VelocityMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for VelocityMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val = 0.0;
        for joint in v.joints.iter() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            x_val += (joint_value - v.history.prev1.get_joint_position(&joint.name)).powi(2);
        }
        x_val = x_val.sqrt();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl VelocityMinimizationObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        VelocityMinimizationObjective::new(name,weight)
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
        let x_val: f64 = (state.origin.translation.vector - past).norm().powi(2);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OriginVelocityMinimizationObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        OriginVelocityMinimizationObjective::new(name,weight)
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
pub struct AccelerationMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl AccelerationMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for AccelerationMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val = 0.0;
        for joint in v.joints.iter() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            let v1: f64 = joint_value - v.history.prev1.get_joint_position(&joint.name);
            let v2: f64 = v.history.prev1.get_joint_position(&joint.name)
                - v.history.prev2.get_joint_position(&joint.name);
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl AccelerationMinimizationObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        AccelerationMinimizationObjective::new(name,weight)
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
        let pos1 = state.origin.translation.vector;
        let pos2: Vector3<f64> = v.history.prev1.origin.translation.vector;
            let pos3: Vector3<f64> = v.history.prev2.origin.translation.vector;
            let x_val: f64 = ((pos1 - pos2) - (pos2 - pos3)).norm().powi(2);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OriginAccelerationMinimizationObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        OriginAccelerationMinimizationObjective::new(name,weight)
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
pub struct JerkMinimizationObjective {
    pub name: String,
    pub weight: f64,
}

impl JerkMinimizationObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight }
    }
}

impl Callable<bool> for JerkMinimizationObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let mut x_val = 0.0;
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
            x_val += ((v1 - v2) - (v2 - v3)).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("JerkMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JerkMinimizationObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        JerkMinimizationObjective::new(name,weight)
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
        let pos1 = state.origin.translation.vector;
        let pos2: Vector3<f64> = v.history.prev1.origin.translation.vector;
        let pos3: Vector3<f64> = v.history.prev2.origin.translation.vector;
        let pos4: Vector3<f64> = v.history.prev3.origin.translation.vector;
        x_val = (((pos1 - pos2) - (pos2 - pos3)) - ((pos2 - pos3) - (pos3 - pos4)))
            .norm()
            .powi(2);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OriginJerkMinimizationObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        OriginJerkMinimizationObjective::new(name,weight)
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
// A macro wrapping the velocity/acceleration/jerk minimization objectives with sensible defaults.
pub struct SmoothnessMacroObjective {
    pub name: String,
    pub weight: f64,
    #[serde(
        skip_serializing,
        default = "SmoothnessMacroObjective::default_velocity_objective"
    )]
    velocity_objective: VelocityMinimizationObjective,
    #[serde(
        skip_serializing,
        default = "SmoothnessMacroObjective::default_acceleration_objective"
    )]
    acceleration_objective: AccelerationMinimizationObjective,
    #[serde(
        skip_serializing,
        default = "SmoothnessMacroObjective::default_jerk_objective"
    )]
    jerk_objective: JerkMinimizationObjective,
    #[serde(
        skip_serializing,
        default = "SmoothnessMacroObjective::default_origin_velocity_objective"
    )]
    base_velocity_objective: OriginVelocityMinimizationObjective,
    #[serde(
        skip_serializing,
        default = "SmoothnessMacroObjective::default_origin_acceleration_objective"
    )]
    base_acceleration_objective: OriginAccelerationMinimizationObjective,
    #[serde(
        skip_serializing,
        default = "SmoothnessMacroObjective::default_origin_jerk_objective"
    )]
    base_jerk_objective: OriginJerkMinimizationObjective,
}

impl SmoothnessMacroObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self {
            name: name.clone(),
            weight,
            velocity_objective: VelocityMinimizationObjective::new(
                format!("Macro {} Velocity", name),
                0.21,
            ),
            acceleration_objective: AccelerationMinimizationObjective::new(
                format!("Macro {} Accel", name),
                0.08,
            ),
            jerk_objective: JerkMinimizationObjective::new(format!("Macro {} Jerk", name), 0.04),
            base_velocity_objective: OriginVelocityMinimizationObjective::new(
                format!("Macro {} Origin Velocity", name),
                0.47,
            ),
            base_acceleration_objective: OriginAccelerationMinimizationObjective::new(
                format!("Macro {} Origin Accel", name),
                0.15,
            ),
            base_jerk_objective: OriginJerkMinimizationObjective::new(
                format!("Macro {} Origin Jerk", name),
                0.05,
            ),
        }
    }

    pub fn default_velocity_objective() -> VelocityMinimizationObjective {
        VelocityMinimizationObjective::new("Macro Velocity".to_string(), 0.21)
    }

    pub fn default_acceleration_objective() -> AccelerationMinimizationObjective {
        AccelerationMinimizationObjective::new("Macro Acceleration".to_string(), 0.08)
    }

    pub fn default_jerk_objective() -> JerkMinimizationObjective {
        JerkMinimizationObjective::new("Macro Jerk".to_string(), 0.04)
    }

    pub fn default_origin_velocity_objective() -> OriginVelocityMinimizationObjective {
        OriginVelocityMinimizationObjective::new("Macro Origin Velocity".to_string(), 0.47)
    }

    pub fn default_origin_acceleration_objective() -> OriginAccelerationMinimizationObjective {
        OriginAccelerationMinimizationObjective::new("Macro Origin Acceleration".to_string(), 0.15)
    }

    pub fn default_origin_jerk_objective() -> OriginJerkMinimizationObjective {
        OriginJerkMinimizationObjective::new("Macro Origin Jerk".to_string(), 0.05)
    }
}

impl Callable<bool> for SmoothnessMacroObjective {
    fn call(&self, v: &Vars, state: &State) -> f64 {
        let velocity_cost = self.velocity_objective.call(v, state);
        let acceleration_cost = self.acceleration_objective.call(v, state);
        let jerk_cost = self.jerk_objective.call(v, state);
        let base_velocity_cost = self.base_velocity_objective.call(v, state);
        let base_acceleration_cost = self.base_acceleration_objective.call(v, state);
        let base_jerk_cost = self.base_jerk_objective.call(v, state);
        return self.weight
            * (velocity_cost
                + acceleration_cost
                + jerk_cost
                + base_velocity_cost
                + base_acceleration_cost
                + base_jerk_cost);
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl SmoothnessMacroObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        SmoothnessMacroObjective::new(name,weight)
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