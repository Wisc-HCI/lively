use serde::{Serialize, Deserialize};
use nalgebra::geometry::{Isometry3};
use nalgebra::Vector3;
use bevy::prelude::*;

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct BoxShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed
}

impl BoxShape {
    pub fn new(name: String, frame:String, physical:bool, x: f64, y: f64, z: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, physical, x, y, z, local_transform}
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CylinderShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub length: f64,
    pub radius: f64,
    pub local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl CylinderShape {
    pub fn new(name: String, frame:String, physical:bool, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, physical, length, radius, local_transform}
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CapsuleShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub length: f64,
    pub radius: f64,
    pub local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl CapsuleShape {
    pub fn new(name: String, frame:String, physical:bool, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, physical, length, radius, local_transform}
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SphereShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub radius: f64,
    pub local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl SphereShape {
    pub fn new(name: String, frame:String, physical:bool, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, physical, radius, local_transform}
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct MeshShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub filename: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub local_transform: Isometry3<f64>
}

impl MeshShape {
    pub fn new(name: String, frame:String, physical:bool, filename: String, x: f64, y: f64, z: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, physical, filename, x, y, z, local_transform}
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct HullShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub points: Vec<Vector3<f64>>,
    pub local_transform: Isometry3<f64>
}

impl HullShape {
    pub fn new(name: String, frame:String, physical:bool, points: Vec<Vector3<f64>>, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, physical, points, local_transform}
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,PartialEq,Debug)]
#[serde(tag = "type")]
pub enum Shape {
    Box(BoxShape),
    Cylinder(CylinderShape),
    Sphere(SphereShape),
    Capsule(CapsuleShape),
    Mesh(MeshShape),
    Hull(HullShape)
}

// impl Into<Mesh> for Shape {
//     fn into(self: &Shape) -> Mesh {
//         match self {
//             Shape::Box(boxShape) => {
                
//             },
//             Shape::Cylinder(boxShape) => {
                
//             },
//             Shape::Sphere(boxShape) => {
                
//             },
//             Shape::Capsule(boxShape) => {
                
//             },
//             Shape::Mesh(boxShape) => {
                
//             },
//             Shape::Hull(hullShape) => {

//             }
//         }
//     }
// }