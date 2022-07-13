use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use nalgebra::geometry::{Isometry3};
use nalgebra::{Vector3};
use crate::utils::info::*;

/*
A read-only struct that provides information about the origin, jointstate, and frames of a robot.
*/

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(rename_all = "camelCase")]
pub struct State {
    pub origin: Isometry3<f64>,
    pub joints: HashMap<String,f64>,
    #[serde(skip_deserializing)]
    pub frames: HashMap<String,Isometry3<f64>>,
    #[serde(skip_deserializing)]
    pub proximity: Vec<ProximityInfo>,
    #[serde(skip_deserializing)]
    pub center_of_mass: Vector3<f64>,
    #[serde(skip)]
    default_joint_position: f64,
    #[serde(skip,default="Isometry3::identity")]
    default_frame_transform: Isometry3<f64>
}

impl State {
    pub fn new(
        origin: Isometry3<f64>, 
        joints: HashMap<String,f64>, 
        frames: HashMap<String,Isometry3<f64>>, 
        proximity: Vec<ProximityInfo>,
        center_of_mass: Vector3<f64>
    ) -> Self {
        Self { origin, joints, 
            frames, proximity,
            center_of_mass,
            default_joint_position: 0.0,
            default_frame_transform: Isometry3::identity() }
    }

    pub fn get_link_transform(&self, link: &String) -> Isometry3<f64> {
        return *self.frames.get(link).unwrap_or(&self.default_frame_transform)
    }

    pub fn get_joint_position(&self, joint: &String) -> f64 {
        return *self.joints.get(joint).unwrap_or(&self.default_joint_position)
    }
}