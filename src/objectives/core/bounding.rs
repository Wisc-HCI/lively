use pyo3::prelude::*;
use crate::utils::vars::Vars;
use crate::utils::state::State;
use crate::objectives::objective::groove_loss;
use nalgebra::geometry::{Isometry3, Translation3, UnitQuaternion};
use nalgebra::{Vector3, vector};

#[pyclass]
#[derive(Clone,Debug)]
pub struct PositionBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String,
    // Goal Value
    pub goal: (Translation3<f64>,UnitQuaternion<f64>,Vector3<f64>)
}
#[pymethods]
impl PositionBoundingObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {name, weight, link, goal: (Translation3::new(0.0,0.0,0.0),UnitQuaternion::identity(),vector![0.0,0.0,0.0])}
    }
}
impl PositionBoundingObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let position = state.get_link_transform(&self.link).translation.vector;
        let transform = Isometry3::from_parts(self.goal.0, self.goal.1);
        let pos = transform.inverse_transform_vector(&position);
        let x_val = (pos[0].powi(1) / self.goal.2[0].powi(2)
                    + pos[1].powi(2) / self.goal.2[1].powi(2)
                    + pos[2].powi(2) / self.goal.2[2].powi(2))
                .powi(2);
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct OrientationBoundingObjective {
    // Bounds the orientation within a region on the unit sphere according to the input provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String,
    // Goal Value
    pub goal: (UnitQuaternion<f64>,f64)
}
#[pymethods]
impl OrientationBoundingObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {name, weight, link, goal: (UnitQuaternion::identity(),0.0)}
    }
}
impl OrientationBoundingObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let orientation = state.get_link_transform(&self.link).rotation;
        let angle_dist = orientation.angle_to(&self.goal.0);
        let x_val = (angle_dist-self.goal.1).max(0.0);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct JointBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub joint: String,
    // Goal Value
    pub goal: (f64,f64)
}
#[pymethods]
impl JointBoundingObjective {
    #[new]
    pub fn new(name: String, weight: f64, joint: String) -> Self {
        Self {name, weight, joint, goal: (0.0,0.0)}
    }
}
impl JointBoundingObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let joint_value = state.get_joint_position(&self.joint);
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        if self.goal.1 <= 0.0 {
            let x_val: f64 = a * (joint_value - self.goal.0).abs().powi(50);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
        } else {
            let l: f64 = self.goal.0 - self.goal.1;
            let u: f64 = self.goal.0 + self.goal.1;
            let r: f64 = (joint_value - l) / (u - l);
            let n: f64 = 2.0 * (r - 0.5);
            let x_val: f64 = a * n.powi(50);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
        }
    }
}
