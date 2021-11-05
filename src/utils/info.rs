use serde::{Serialize, Deserialize};
use urdf_rs::{Mimic};
// use std::fmt::Display;

#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct MimicInfo {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64
}

impl MimicInfo {
    pub fn new(joint: String, multiplier: f64, offset: f64) -> Self {
        Self { joint, multiplier, offset }
    }
}

impl From<Mimic> for MimicInfo {
    fn from(mimic: Mimic) -> Self {
        let joint: String = mimic.joint;
        let multiplier: f64;
        let offset: f64;
        match mimic.multiplier {
            Some(value) => multiplier = value,
            None => multiplier = 1.0
        };
        match mimic.offset {
            Some(value) => offset = value,
            None => offset = 0.0
        }
        MimicInfo { joint, multiplier, offset }
    }
}

#[derive(Serialize,Deserialize,Clone,Debug,Default)]
#[serde(rename_all = "camelCase")]
pub struct JointInfo {
    pub name: String,
    pub joint_type: String,
    pub lower_bound: f64,
    pub upper_bound: f64,
    pub max_velocity: f64,
    pub axis: [f64; 3],
    pub mimic: Option<MimicInfo>,

    // Pure utility value
    pub idx: usize
}

impl JointInfo {
    pub fn new(name: String, joint_type: String, lower_bound: f64, upper_bound: f64, max_velocity: f64, axis: [f64; 3], mimic: Option<MimicInfo>) -> Self {
        Self { name, joint_type, lower_bound, upper_bound, max_velocity, axis, mimic, idx: 0 }
    }
}

#[derive(Serialize,Deserialize,Clone,Debug,Default)]
#[serde(rename_all = "camelCase")]
pub struct LinkInfo {
    pub name: String,
    pub parent_joint: String,
}

impl LinkInfo {
    pub fn new(name: String, parent_joint: String) -> Self {
        Self { name, parent_joint }
    }
}

#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct ProximityInfo {
    pub shape1: String,
    pub shape2: String,
    pub distance: Option<f64>,
    pub physical: bool
}

impl ProximityInfo {
    pub fn new(shape1: String, shape2: String, distance: Option<f64>, physical: bool) -> Self {
        Self { shape1, shape2, distance, physical }
    }
}