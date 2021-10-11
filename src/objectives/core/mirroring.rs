use pyo3::prelude::*;
use crate::utils::goals::*;
use crate::utils::transformations::*;
use crate::utils::vars::Vars;
use crate::objectives::objective::{groove_loss};
use nalgebra::geometry::{Quaternion, UnitQuaternion};
use nalgebra::{Vector3};

#[pyclass]
#[derive(Clone,Debug)]
pub struct PositionMirroring {
    // Matches the position between two joints, with a difference according to the Vector3 provided in goals.
    #[pyo3(get)]
    pub name: String
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link1: String
    #[pyo3(get)]
    pub link2: String
    // Goal Value
    pub goal: Vector3
}
#[pymethods]
impl PositionMirroring {
    #[new]
    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: vector![0.0,0.0,0.0]}
    }
}
impl PositionMirroring {
    fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        const link1_translation = state.get_link_transform(&self.link1).translation.vector;
        const link2_translation = state.get_link_transform(&self.link2).translation.vector;
        const x_val = ((link1_translation - link2_translation) - self.goal).norm();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct OrientationMirroring {
    // Matches the orientation between two joints, with a difference according to the Quaternion provided in goals.
    #[pyo3(get)]
    pub name: String
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link1: String
    #[pyo3(get)]
    pub link2: String
    // Goal Value
    pub goal: UnitQuaternion
}
#[pymethods]
impl OrientationMirroring {
    #[new]
    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: UnitQuaternion::identity()}
    }
}
impl OrientationMirroring {
    fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        const link1_rotation = state.get_link_transform(&self.link1).rotation;
        const link2_rotation = state.get_link_transform(&self.link2).rotation;
        const x_val = link1_rotation.rotation_to(link2_rotation).angle_to(self.goal)
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct JointMirroring {
    // Match joint values according to the difference specified in goals
    #[pyo3(get)]
    pub name: String
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub joint1: String
    #[pyo3(get)]
    pub joint2: String
    // Goal Value
    pub goal: f64
}
#[pymethods]
impl JointMirroring {
    #[new]
    pub fn new(name: String, weight: f64, joint1: String, joint2: String) -> Self {
        Self { name, weight, joint1, joint2, goal: 0.0}
    }
}
impl JointMirroring {
    fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        const joint1_position = state.get_joint_position(&self.joint1);
        const joint2_position = state.get_joint_position(&self.joint2);
        const x_val = ((joint1_position+self.goal)-joint2_position).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
    }
}
