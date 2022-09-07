use serde::{Serialize, Deserialize};
use nalgebra::geometry::{Isometry3};
use nalgebra::Vector3;
use bevy::prelude::*;
use bevy::prelude::shape::CapsuleUvProfile;


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

impl Into<Mesh> for Shape {
    fn into(self: Shape) -> Mesh {
        match self {
            Shape::Box(box_object) => {
                return Mesh::from(shape::Box::new(box_object.x as f32 ,box_object.y as f32, box_object.z as f32));
            },
            Shape::Cylinder(cylinder_object) => {
                return Mesh::from(shape::Capsule { radius: cylinder_object.radius as f32, rings:0, depth: cylinder_object.length as f32,
                    latitudes : 0, longitudes : 32 , uv_profile : CapsuleUvProfile::Aspect});
               
            },
            Shape::Sphere(sphere_object) => {
                return Mesh::from(shape::Icosphere {radius : sphere_object.radius as f32, subdivisions : 32});
            },
            Shape::Capsule(capsule_object) => {
                return Mesh::from(shape::Capsule { radius: capsule_object.radius as f32, rings:0, depth: capsule_object.length as f32,
                    latitudes : 16, longitudes : 32 , uv_profile : CapsuleUvProfile::Aspect});
                
            },
            Shape::Mesh(_) => {
                return Mesh::from(shape::Cube{size : 1.0});
            },
            Shape::Hull(_) => {
                // for points in hull_object.points{

                // }
                // let hull_points: Vec<Vec<f32>> = hull_object
                //         .points
                //         .iter()
                //         .map(|p| vec![p.x, p.y, p.z])
                //         .collect();
                // let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
                // mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION,  vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]);
                // mesh.set_indices(Some(Indices::U32(vec![0,1,2])));
                // return mesh;
                return Mesh::from(shape::Cube{size : 1.0});
                 
            }
        }
    }
}