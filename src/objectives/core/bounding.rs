use crate::objectives::objective::{groove_loss, Callable};
use crate::utils::state::State;
use crate::utils::vars::Vars;
use nalgebra::geometry::{Isometry3, UnitQuaternion};
use nalgebra::{vector, Vector3};
use serde::{Deserialize, Serialize};

fn get_default_pos_bound() -> (Isometry3<f64>, Vector3<f64>) {
    return (Isometry3::identity(), vector![0.0, 0.0, 0.0]);
}

fn get_default_rot_bound() -> (UnitQuaternion<f64>, f64) {
    return (UnitQuaternion::identity(), 0.0);
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PositionBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip, default = "get_default_pos_bound")]
    pub goal: (Isometry3<f64>, Vector3<f64>),
}

impl PositionBoundingObjective {
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {
            name,
            weight,
            link,
            goal: (Isometry3::identity(), vector![0.0, 0.0, 0.0]),
        }
    }
}

impl Callable<(Isometry3<f64>, Vector3<f64>)> for PositionBoundingObjective {
    fn call(&self, _v: &Vars, state: &State, _is_core: bool) -> f64 {
        let position = state.get_link_transform(&self.link).translation.vector;
        let pos = self.goal.0.inverse_transform_point(&position.into());
        let dist = pos[0].powi(2) / self.goal.1[0].powi(2)
            + pos[1].powi(2) / self.goal.1[1].powi(2)
            + pos[2].powi(2) / self.goal.1[2].powi(2);
        let cost = 1.0 / (1.0 + (-2.0 * dist + 4.0).exp()) + dist / 10.0;
        return self.weight * cost; //groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn set_goal(&mut self, goal: (Isometry3<f64>, Vector3<f64>)) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct OrientationBoundingObjective {
    // Bounds the orientation within a region on the unit sphere according to the input provided in goals.
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip, default = "get_default_rot_bound")]
    pub goal: (UnitQuaternion<f64>, f64),
}

impl OrientationBoundingObjective {
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {
            name,
            weight,
            link,
            goal: (UnitQuaternion::identity(), 0.0),
        }
    }
}

impl Callable<(UnitQuaternion<f64>, f64)> for OrientationBoundingObjective {
    fn call(&self, _v: &Vars, state: &State, _is_core: bool) -> f64 {
        let orientation = state.get_link_transform(&self.link).rotation;
        let angle_dist = orientation.angle_to(&self.goal.0);
        let x_val = (angle_dist - self.goal.1).max(0.0);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_goal(&mut self, goal: (UnitQuaternion<f64>, f64)) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct JointBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    pub name: String,
    pub weight: f64,
    pub joint: String,
    // Goal Value
    #[serde(skip)]
    pub goal: (f64, f64),
}

impl JointBoundingObjective {
    pub fn new(name: String, weight: f64, joint: String) -> Self {
        Self {
            name,
            weight,
            joint,
            goal: (0.0, 0.0),
        }
    }
}

impl Callable<(f64, f64)> for JointBoundingObjective {
    fn call(&self, _v: &Vars, state: &State, _is_core: bool) -> f64 {
        let joint_value = state.get_joint_position(&self.joint);
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        if self.goal.1 <= 0.0 {
            let x_val: f64 = a * (joint_value - self.goal.0).abs().powi(50);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2);
        } else {
            let l: f64 = self.goal.0 - self.goal.1;
            let u: f64 = self.goal.0 + self.goal.1;
            let r: f64 = (joint_value - l) / (u - l);
            let n: f64 = 2.0 * (r - 0.5);
            let x_val: f64 = a * n.powi(50);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2);
        }
    }

    fn set_goal(&mut self, goal: (f64, f64)) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}
