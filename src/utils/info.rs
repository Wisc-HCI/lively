use crate::utils::shapes::{BoxShape, CapsuleShape, CylinderShape, MeshShape, Shape, SphereShape};
use k::urdf::isometry_from;
use nalgebra::geometry::Point3;
use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};
use urdf_rs::{Geometry, Link, Mimic};
// use std::time::{Duration};
// use std::fmt::Display;

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct TransformInfo {
    #[serde(skip_deserializing,default="Isometry3::identity")]
    pub world: Isometry3<f64>,
    #[serde(skip_deserializing,default="Isometry3::identity")]
    pub local: Isometry3<f64>
}

impl TransformInfo {
    pub fn new(world: Isometry3<f64>, local: Isometry3<f64>) -> Self {
        Self { world, local }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct MimicInfo {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64,
}

impl MimicInfo {
    pub fn new(joint: String, multiplier: f64, offset: f64) -> Self {
        Self {
            joint,
            multiplier,
            offset,
        }
    }
}

impl From<&Mimic> for MimicInfo {
    fn from(mimic: &Mimic) -> Self {
        let joint: String = mimic.joint.clone();
        let multiplier: f64;
        let offset: f64;
        match mimic.multiplier {
            Some(value) => multiplier = value,
            None => multiplier = 1.0,
        };
        match mimic.offset {
            Some(value) => offset = value,
            None => offset = 0.0,
        }
        MimicInfo {
            joint,
            multiplier,
            offset,
        }
    }
}


#[repr(C)]
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
    pub idx: usize,
}

impl JointInfo {
    pub fn new(
        name: String,
        joint_type: String,
        lower_bound: f64,
        upper_bound: f64,
        max_velocity: f64,
        axis: [f64; 3],
        mimic: Option<MimicInfo>,
    ) -> Self {
        Self {
            name,
            joint_type,
            lower_bound,
            upper_bound,
            max_velocity,
            axis,
            mimic,
            idx: 0,
        }
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
#[serde(rename_all = "camelCase")]
pub struct LinkInfo {
    pub name: String,
    pub parent_joint: String,
    pub visuals: Vec<Shape>,
    pub collisions: Vec<Shape>,
}

impl LinkInfo {
    pub fn new(
        name: String,
        parent_joint: String,
        visuals: Vec<Shape>,
        collisions: Vec<Shape>,
    ) -> Self {
        Self {
            name,
            parent_joint,
            visuals,
            collisions,
        }
    }
}

impl From<Link> for LinkInfo {
    fn from(link: Link) -> Self {
        let name: String = link.name.clone();
        let parent_joint: String = String::from("world"); // Override later if needed
        let visuals: Vec<Shape> = link
            .visual
            .iter()
            .map(|visual| match &visual.geometry {
                Geometry::Box { size } => Shape::Box(BoxShape::new(
                    visual
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    size[0],
                    size[1],
                    size[2],
                    isometry_from(&visual.origin),
                )),
                Geometry::Sphere { radius } => Shape::Sphere(SphereShape::new(
                    visual
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    radius.clone(),
                    isometry_from(&visual.origin),
                )),
                Geometry::Cylinder { radius, length } => Shape::Cylinder(CylinderShape::new(
                    visual
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    length.clone(),
                    radius.clone(),
                    isometry_from(&visual.origin),
                )),
                Geometry::Capsule { radius, length } => Shape::Capsule(CapsuleShape::new(
                    visual
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    length.clone(),
                    radius.clone(),
                    isometry_from(&visual.origin),
                )),
                Geometry::Mesh { filename, scale } => {
                    let size = scale.unwrap_or([1.0, 1.0, 1.0]);
                    return Shape::Mesh(MeshShape::new(
                        visual
                            .name
                            .as_ref()
                            .unwrap_or(&link.name.clone())
                            .to_string(),
                        link.name.clone(),
                        true,
                        filename.clone(),
                        size[0],
                        size[1],
                        size[2],
                        isometry_from(&visual.origin),
                    ));
                }
            })
            .collect();
        let collisions: Vec<Shape> = link
            .collision
            .iter()
            .map(|collision| match &collision.geometry {
                Geometry::Box { size } => Shape::Box(BoxShape::new(
                    collision
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    size[0],
                    size[1],
                    size[2],
                    isometry_from(&collision.origin),
                )),
                Geometry::Sphere { radius } => Shape::Sphere(SphereShape::new(
                    collision
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    radius.clone(),
                    isometry_from(&collision.origin),
                )),
                Geometry::Cylinder { radius, length } => Shape::Cylinder(CylinderShape::new(
                    collision
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    length.clone(),
                    radius.clone(),
                    isometry_from(&collision.origin),
                )),
                Geometry::Capsule { radius, length } => Shape::Capsule(CapsuleShape::new(
                    collision
                        .name
                        .as_ref()
                        .unwrap_or(&link.name.clone())
                        .to_string(),
                    link.name.clone(),
                    true,
                    length.clone(),
                    radius.clone(),
                    isometry_from(&collision.origin),
                )),
                Geometry::Mesh { filename, scale } => {
                    let size = scale.unwrap_or([1.0, 1.0, 1.0]);
                    return Shape::Mesh(MeshShape::new(
                        collision
                            .name
                            .as_ref()
                            .unwrap_or(&link.name.clone())
                            .to_string(),
                        link.name.clone(),
                        true,
                        filename.clone(),
                        size[0],
                        size[1],
                        size[2],
                        isometry_from(&collision.origin),
                    ));
                }
            })
            .collect();
        return Self {
            name,
            parent_joint,
            visuals,
            collisions,
        };
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct ProximityInfo {
    pub shape1: String,
    pub shape2: String,
    pub distance: f64,
    pub points: Option<(Point3<f64>, Point3<f64>)>,
    pub physical: bool,
    pub loss : f64,
   // pub average_distance : Option<f64>
}

impl ProximityInfo {
    pub fn new(
        shape1: String,
        shape2: String,
        distance: f64,
        points: Option<(Point3<f64>, Point3<f64>)>,
        physical: bool,
        loss : f64,
       
    ) -> Self {
        Self {
            shape1,
            shape2,
            distance,
            points,
            physical,
            loss, 
          

        }
    }
}


#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct CollisionSettingInfo {
    pub d_max: f64,
    pub r: f64,
    pub a_max: f64,
    pub time_budget: f64,
    pub timed: bool,
    pub a_value: f64,
}



impl CollisionSettingInfo {
    pub fn new(
        d_max: f64,
        r: f64,
        a_max: f64,
        time_budget: f64,
        timed: bool,
        a_value: f64,
    ) -> Self {
        Self {
            d_max,
            r,
            a_max,
            time_budget,
            timed,
            a_value,
        }
    }

}

// const D_MAX: f64 = 1.0;
// const R: f64 = 0.0;
// const A_MAX: f64 = 0.5;
// const TIME_BUDGET: Duration = Duration::from_micros(100);

// const TIMED: bool = true;
// const A_VALUE: f64 = 1.0;
// const OPTIMA_NUMBER: usize = 600;

impl Default for CollisionSettingInfo {
    fn default() -> Self {
        Self { d_max : 1.0,  r : 0.0 , a_max: 0.5, time_budget : 0.1, timed : true, a_value : 1.0  }
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum ShapeUpdate {
    Add { id: String, shape: Shape },
    Move { id: String, pose: Isometry3<f64> },
    Delete(String),
}
