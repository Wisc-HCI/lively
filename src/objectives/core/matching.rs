use pyo3::prelude::*;
use crate::utils::vars::Vars;
use crate::utils::state::State;
use crate::objectives::objective::groove_loss;
use nalgebra::geometry::{UnitQuaternion};
use nalgebra::{Vector3, vector};

#[pyclass]
#[derive(Clone,Debug)]
pub struct PositionMatchObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String,
    // Goal Value
    pub goal: Vector3<f64>
}

#[pymethods]
impl PositionMatchObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self { name, weight, link, goal: vector![0.0,0.0,0.0] }
    }
}

impl PositionMatchObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        // Get the link transform from frames
        let link_translation = state.get_link_transform(&self.link).translation.vector;

        let x_val = (link_translation - self.goal).norm();

        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct OrientationMatchObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String,
    // Goal Value
    pub goal: UnitQuaternion<f64>
}
#[pymethods]
impl OrientationMatchObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self { name, weight, link, goal: UnitQuaternion::identity() }
    }
}

impl OrientationMatchObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {

        // Get the link transform from frames
        let link_rotation = state.get_link_transform(&self.link).rotation;

        let x_val = link_rotation.angle_to(&self.goal);

        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct JointMatchObjective {
    // Sets a joint to a value given in scalar goal
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub joint: String,
    // Goal Value
    pub goal: f64
}
#[pymethods]
impl JointMatchObjective {
    #[new]
    pub fn new(name: String, weight: f64, joint: String) -> Self {
        Self { name, weight, joint, goal: 0.0 }
    }
}
impl JointMatchObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let x_val = (self.goal - state.get_joint_position(&self.joint)).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct OriginPositionMatchObjective {
    // Adds position liveliness to the Origin node (first three entries in x are these values)
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    // Goal Value
    pub goal: Vector3<f64>
}
#[pymethods]
impl OriginPositionMatchObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight, goal: vector![0.0,0.0,0.0]}
    }
}
impl OriginPositionMatchObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let x_val = (self.goal - state.origin.translation.vector).norm();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct OriginOrientationMatchObjective {
    // Adds Orientation liveliness to the Origin node (first three entries in x are these values)
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    // Goal Value
    pub goal: UnitQuaternion<f64>
}
#[pymethods]
impl OriginOrientationMatchObjective {
    #[new]
    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight, goal: UnitQuaternion::identity()}
    }
}
impl OriginOrientationMatchObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let x_val = state.origin.rotation.angle_to(&self.goal);
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct DistanceMatchObjective {
    // Specify that the cartesian distance between two links is maintained
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link1: String,
    #[pyo3(get)]
    pub link2: String,
    // Goal Value
    pub goal: f64
}
#[pymethods]
impl DistanceMatchObjective{
    #[new]
    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: 0.0}
    }
}
impl DistanceMatchObjective {
    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let link1_translation = state.get_link_transform(&self.link1).translation.vector;
        let link2_translation = state.get_link_transform(&self.link2).translation.vector;
        let x_val = ((link1_translation-link2_translation).norm() - self.goal).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}
