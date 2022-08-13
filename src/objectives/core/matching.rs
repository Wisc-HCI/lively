use serde::{Serialize, Deserialize};
use crate::utils::vars::Vars;
use crate::utils::state::State;
use crate::objectives::objective::groove_loss;
use nalgebra::geometry::{UnitQuaternion};
use nalgebra::{Vector3, vector};

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct PositionMatchObjective {
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip)]
    pub goal: Vector3<f64>
}

impl PositionMatchObjective {

    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self { name, weight, link, goal: vector![0.0,0.0,0.0] }
    }

    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {
        // Get the link transform from frames
        let link_translation = state.get_link_transform(&self.link).translation.vector;

        let x_val = (link_translation - self.goal).norm();

        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct OrientationMatchObjective {
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip)]
    pub goal: UnitQuaternion<f64>
}

impl OrientationMatchObjective {

    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self { name, weight, link, goal: UnitQuaternion::identity() }
    }

    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {

        // Get the link transform from frames
        let link_rotation = state.get_link_transform(&self.link).rotation;

        let x_val = link_rotation.angle_to(&self.goal);

        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct JointMatchObjective {
    // Sets a joint to a value given in scalar goal
    pub name: String,
    pub weight: f64,
    pub joint: String,
    // Goal Value
    #[serde(skip)]
    pub goal: f64
}

impl JointMatchObjective {

    pub fn new(name: String, weight: f64, joint: String) -> Self {
        Self { name, weight, joint, goal: 0.0 }
    }

    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {
        let x_val = (self.goal - state.get_joint_position(&self.joint)).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct OriginPositionMatchObjective {
    // Adds position liveliness to the Origin node (first three entries in x are these values)
    pub name: String,
    pub weight: f64,
    // Goal Value
    #[serde(skip)]
    pub goal: Vector3<f64>
}

impl OriginPositionMatchObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight, goal: vector![0.0,0.0,0.0]}
    }

    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {
        let x_val = (self.goal - state.origin.translation.vector).norm();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct OriginOrientationMatchObjective {
    // Adds Orientation liveliness to the Origin node (first three entries in x are these values)
    pub name: String,
    pub weight: f64,
    // Goal Value
    #[serde(skip)]
    pub goal: UnitQuaternion<f64>
}

impl OriginOrientationMatchObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self { name, weight, goal: UnitQuaternion::identity()}
    }

    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {
        let x_val = state.origin.rotation.angle_to(&self.goal);
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct DistanceMatchObjective {
    // Specify that the cartesian distance between two links is maintained
    pub name: String,
    pub weight: f64,
    pub link1: String,
    pub link2: String,
    // Goal Value
    #[serde(skip)]
    pub goal: f64
}

impl DistanceMatchObjective {

    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: 0.0}
    }

    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {
        let link1_translation = state.get_link_transform(&self.link1).translation.vector;
        let link2_translation = state.get_link_transform(&self.link2).translation.vector;
        let x_val = ((link1_translation-link2_translation).norm() - self.goal).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}
