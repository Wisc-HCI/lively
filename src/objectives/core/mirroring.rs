use pyo3::prelude::*;
use crate::utils::vars::Vars;
use crate::utils::state::State;
use crate::objectives::objective::groove_loss;
use nalgebra::geometry::{UnitQuaternion};
use nalgebra::{Vector3, vector};

#[pyclass]
#[derive(Clone,Debug)]
pub struct PositionMirroringObjective {
    // Matches the position between two joints, with a difference according to the Vector3 provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link1: String,
    #[pyo3(get)]
    pub link2: String,
    // Goal Value
    pub goal: Vector3<f64>
}
#[pymethods]
impl PositionMirroringObjective {
    #[new]
    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: vector![0.0,0.0,0.0]}
    }
}
impl PositionMirroringObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let link1_translation = state.get_link_transform(&self.link1).translation.vector;
        let link2_translation = state.get_link_transform(&self.link2).translation.vector;
        let x_val = ((link1_translation - link2_translation) - self.goal).norm();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct OrientationMirroringObjective {
    // Matches the orientation between two joints, with a difference according to the Quaternion provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link1: String,
    #[pyo3(get)]
    pub link2: String,
    // Goal Value
    pub goal: UnitQuaternion<f64>
}
#[pymethods]
impl OrientationMirroringObjective {
    #[new]
    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: UnitQuaternion::identity()}
    }
}
impl OrientationMirroringObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let link1_rotation = state.get_link_transform(&self.link1).rotation;
        let link2_rotation = state.get_link_transform(&self.link2).rotation;
        let x_val = link1_rotation.rotation_to(&link2_rotation).angle_to(&self.goal);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct JointMirroringObjective {
    // Match joint values according to the difference specified in goals
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub joint1: String,
    #[pyo3(get)]
    pub joint2: String,
    // Goal Value
    pub goal: f64
}
#[pymethods]
impl JointMirroringObjective {
    #[new]
    pub fn new(name: String, weight: f64, joint1: String, joint2: String) -> Self {
        Self { name, weight, joint1, joint2, goal: 0.0}
    }
}
impl JointMirroringObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let joint1_position = state.get_joint_position(&self.joint1);
        let joint2_position = state.get_joint_position(&self.joint2);
        let x_val = ((joint1_position+self.goal)-joint2_position).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
    }
}
