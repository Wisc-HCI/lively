use pyo3::prelude::*;
use nalgebra::geometry::{Isometry3};
use nalgebra::{one, Vector3};
use crate::objectives::objective::groove_loss;
use crate::vars::state::State;


// ======= JointLimitsObjective ======= 
#[pyclass]
#[derive(Clone,Debug)]
pub struct JointLimitsObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64
}

impl JointLimitsObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for joint in v.joints.iter() {
            let l = joint.lower_bound;
            let u = joint.upper_bound;
            let joint_value = state.get_joint_position(&joint.name);
            if u - l <= 0.0 {
                // In cases where the upper and lower limits are the same,
                // just compare the lower limit to the x value.
                sum += a * (joint_value - l).abs().powi(50);
            } else {
                // Otherwise, compare as normal
                let r = (joint_value - l) / (u - l);
                let n = 2.0 * (r - 0.5);
                sum += a * n.powi(50);
            }
        }
        // println!("JointLimits error: {:?}",sum);
        return self.weight * groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
}

#[pymethods]
impl JointLimitsObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

// ======= VelocityMinimizationObjective ======= 
#[pyclass]
#[derive(Clone,Debug)]
pub struct VelocityMinimizationObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64
}

impl VelocityMinimizationObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for (i,joint) in v.joints.iter().enumerate() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            if is_core {
                x_val += (joint_value - v.history_core.prev1.get_joint_position(&joint.name)).powi(2);
            } else {
                x_val += (joint_value - v.history.prev1.get_joint_position(&joint.name)).powi(2);
            }
        }
        x_val = x_val.sqrt();
        // println!("VelocityMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pymethods]
impl VelocityMinimizationObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct BaseVelocityMinimizationObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64
}

impl BaseVelocityMinimizationObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        let past:Vector3<f64>;
        let x_val:f64;
        if is_core {
            past = v.history_core.prev1.origin.translation 
        } else {
            past = v.history.prev1.origin.translation
        }
        x_val = (state.origin.translation.vector-past.vector).norm().powi(2);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pymethods]
impl BaseVelocityMinimizationObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct AccelerationMinimizationObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64
}

impl AccelerationMinimizationObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for (i,joint) in v.joints.iter().enumerate() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            let v1: f64;
            let v2: f64;
            if is_core {
                v1 = joint_value - v.history_core.prev1.get_joint_position(&joint.name);
                v2 = v.history_core.prev1.get_joint_position(&joint.name) - v.history_core.prev2.get_joint_position(&joint.name);
            } else {
                v1 = joint_value - v.history.prev1.get_joint_position(&joint.name);
                v2 = v.history.prev1.get_joint_position(&joint.name) - v.history.prev2.get_joint_position(&joint.name);
            }
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("AccelerationMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pymethods]
impl AccelerationMinimizationObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct BaseAccelerationMinimizationObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64
}

impl BaseAccelerationMinimizationObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        let x_val: f64;
        let pos1 = origin.translation.vector;
        let pos2:Vector3<f64>;
        let pos3:Vector3<f64>;
        if is_core {
            pos2 = v.history_core.prev1.origin.translation.vector;
            pos3 = v.history_core.prev2.origin.translation.vector;
        } else {
            pos2 = v.history.prev1.origin.translation.vector;
            pos3 = v.history.prev2.origin.translation.vector;
        }
        x_val = ((pos1-pos2)-(pos2-pos3)).norm().powi(2);
        // println!("AccelerationMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pymethods]
impl AccelerationMinimizationObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct JerkMinimizationObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64
}

impl JerkMinimizationObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for (i,joint) in v.joints.iter().enumerate() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            let v1: f64;
            let v2: f64;
            let v3: f64;
            if is_core {
                v1 = joint_value - v.history_core.prev1.get_joint_position(&joint.name);
                v2 = v.history_core.prev1.get_joint_position(&joint.name) - v.history_core.prev2.get_joint_position(&joint.name);
                v3 = v.history_core.prev2.get_joint_position(&joint.name) - v.history_core.prev3.get_joint_position(&joint.name);
            } else {
                v1 = joint_value - v.history.prev1.get_joint_position(&joint.name);
                v2 = v.history.prev1.get_joint_position(&joint.name) - v.history.prev2.get_joint_position(&joint.name);
                v3 = v.history.prev2.get_joint_position(&joint.name) - v.history.prev3.get_joint_position(&joint.name);
            }
            x_val += ((v1 - v2) - (v2 - v3)).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("JerkMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pymethods]
impl JerkMinimizationObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct BaseJerkMinimizationObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64
}

impl BaseJerkMinimizationObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        let x_val: f64;
        let pos1 = origin.translation.vector;
        let pos2:Vector3<f64>;
        let pos3:Vector3<f64>;
        let pos4:Vector3<f64>;
        if is_core {
            pos2 = v.history_core.prev1.origin.translation.vector;
            pos3 = v.history_core.prev2.origin.translation.vector;
            pos4 = v.history_core.prev3.origin.translation.vector;
        } else {
            pos2 = v.history.prev1.origin.translation.vector;
            pos3 = v.history.prev2.origin.translation.vector;
            pos4 = v.history.prev3.origin.translation.vector;
        }
        x_val = (((pos1-pos2) - (pos2-pos3)) - ((pos2-pos3) - (pos3-pos4))).norm().powi(2);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pymethods]
impl BaseJerkMinimizationObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct SmoothnessMacroObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    velocity_objective: VelocityMinimizationObjective,
    acceleration_objective: AccelerationMinimizationObjective,
    jerk_objective: JerkMinimizationObjective,
    base_velocity_objective: BaseVelocityMinimizationObjective,
    base_acceleration_objective: BaseAccelerationMinimizationObjective,
    base_jerk_objective: BaseJerkMinimizationObjective
}

#[pymethods]
impl SmoothnessMacroObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self {
            name,
            weight,
            velocity_objective: VelocityMinimizationObjective::new(name,0.21),
            acceleration_objective: AccelerationMinimizationObjective::new(name,0.08),
            jerk_objective: JerkMinimizationObjective::new(name,0.04),
            base_velocity_objective: BaseVelocityMinimizationObjective::new(name,0.47),
            base_acceleration_objective: BaseAccelerationMinimizationObjective::new(name,0.15),
            base_jerk_objective: BaseJerkMinimizationObjective::new(name,0.05)
        }
    }
}

impl SmoothnessMacroObjective {
    fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        const velocity_cost = self.velocity_objective.call(v, state, is_core);
        const acceleration_cost = self.acceleration_objective.call(v, state, is_core);
        const jerk_cost = self.jerk_objective.call(v, state, is_core);
        const base_velocity_cost = self.base_velocity_objective.call(v, state, is_core);
        const base_acceleration_cost = self.base_acceleration_objective.call(v, state, is_core);
        const base_jerk_cost = self.base_jerk_objective.call(v, state, is_core);
        return self.weight * (velocity_cost + acceleration_cost + jerk_cost +
               base_velocity_cost + base_acceleration_cost + base_jerk_cost);
    }
}
