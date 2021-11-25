use serde::{Serialize, Deserialize};
use urdf_rs::{Mimic, Link, Geometry};
use nalgebra::geometry::Point3;
use nalgebra::Isometry3;
use k::urdf::isometry_from;
use crate::utils::shapes::{Shape, BoxShape, CylinderShape, SphereShape, CapsuleShape, MeshShape};
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
    pub visuals: Vec<Shape>,
    pub collisions: Vec<Shape>
}

impl LinkInfo {
    pub fn new(name: String, parent_joint: String, visuals: Vec<Shape>, collisions: Vec<Shape>) -> Self {
        Self { name, parent_joint, visuals, collisions }
    }
}

impl From<Link> for LinkInfo {
    fn from(link: Link) -> Self {
        let name: String = link.name.clone();
        let parent_joint: String = String::from("world"); // Override later if needed
        let visuals: Vec<Shape> = link.visual.iter().map(|visual| match &visual.geometry {
            Geometry::Box{size} => Shape::Box(BoxShape::new(visual.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,size[0],size[1],size[2],isometry_from(&visual.origin))),
            Geometry::Sphere{radius} => Shape::Sphere(SphereShape::new(visual.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,radius.clone(),isometry_from(&visual.origin))),
            Geometry::Cylinder{radius,length} => Shape::Cylinder(CylinderShape::new(visual.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,length.clone(),radius.clone(),isometry_from(&visual.origin))),
            Geometry::Capsule{radius,length} => Shape::Capsule(CapsuleShape::new(visual.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,length.clone(),radius.clone(),isometry_from(&visual.origin))),
            Geometry::Mesh{filename,scale} => {
                let size = scale.unwrap_or([1.0,1.0,1.0]);
                return Shape::Mesh(MeshShape::new(visual.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,filename.clone(),size[0],size[1],size[2],isometry_from(&visual.origin)))
            },
        }).collect();
        let collisions: Vec<Shape> = link.collision.iter().map(|collision| match &collision.geometry {
            Geometry::Box{size} => Shape::Box(BoxShape::new(collision.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,size[0],size[1],size[2],isometry_from(&collision.origin))),
            Geometry::Sphere{radius} => Shape::Sphere(SphereShape::new(collision.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,radius.clone(),isometry_from(&collision.origin))),
            Geometry::Cylinder{radius,length} => Shape::Cylinder(CylinderShape::new(collision.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,length.clone(),radius.clone(),isometry_from(&collision.origin))),
            Geometry::Capsule{radius,length} => Shape::Capsule(CapsuleShape::new(collision.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,length.clone(),radius.clone(),isometry_from(&collision.origin))),
            Geometry::Mesh{filename,scale} => {
                let size = scale.unwrap_or([1.0,1.0,1.0]);
                return Shape::Mesh(MeshShape::new(collision.name.as_ref().unwrap_or(&link.name.clone()).to_string(),link.name.clone(),true,filename.clone(),size[0],size[1],size[2],isometry_from(&collision.origin)))
            },
        }).collect();
        return Self { name, parent_joint, visuals, collisions }
    }
}

#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct ProximityInfo {
    pub shape1: String,
    pub shape2: String,
    pub distance: f64,
    pub points: Option<(Point3<f64>,Point3<f64>)>,
    pub physical: bool
}

impl ProximityInfo {
    pub fn new(shape1: String, shape2: String, distance: f64, points: Option<(Point3<f64>,Point3<f64>)>, physical: bool) -> Self {
        Self { shape1, shape2, distance, points, physical }
    }
}

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum ShapeUpdate {
    Add{
        id: String,
        shape: Shape
    },
    Move{
        id: String,
        pose: Isometry3<f64>
    },
    Delete(String)
}