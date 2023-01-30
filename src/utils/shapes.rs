
#[cfg(feature = "bevy")]
use bevy::math::f32::*;
#[cfg(feature = "bevy")]
use bevy::prelude::shape::*;
#[cfg(feature = "bevy")]
use bevy::prelude::{Mesh, Transform};
use nalgebra::geometry::Isometry3;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use crate::utils::pyutils::*;


#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct BoxShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub local_transform: Isometry3<f64>, // handle: Whatever Parry3D/Rapier handle is needed
}

impl BoxShape {
    pub fn new(
        name: String,
        frame: String,
        physical: bool,
        x: f64,
        y: f64,
        z: f64,
        local_transform: Isometry3<f64>,
    ) -> Self {
        // TODO: add/create shape and handle
        Self {
            name,
            frame,
            physical,
            x,
            y,
            z,
            local_transform,
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl BoxShape {
    #[new]
    pub fn from_python(name:String, frame:String, physical:bool,x:f64,y:f64,z:f64,translation:PyTranslation,rotation:PyRotation) -> Self {
        Self::new(name,frame,physical,x,y,z,Isometry3::from_parts(translation.value, rotation.value))
    }

    #[getter]
    pub fn get_translation(&self) -> PyResult<PyTranslation> {
        Ok(PyTranslation{value:self.local_transform.translation})
    }

    #[getter]
    pub fn get_rotation(&self) -> PyResult<PyRotation> {
        Ok(PyRotation{value:self.local_transform.rotation})
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
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
    pub fn new(
        name: String,
        frame: String,
        physical: bool,
        length: f64,
        radius: f64,
        local_transform: Isometry3<f64>,
    ) -> Self {
        // TODO: add/create shape and handle
        Self {
            name,
            frame,
            physical,
            length,
            radius,
            local_transform,
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl CylinderShape {
    #[new]
    pub fn from_python(name:String, frame:String, physical:bool,length:f64,radius:f64,translation:PyTranslation,rotation:PyRotation) -> Self {
        Self::new(name,frame,physical,length,radius,Isometry3::from_parts(translation.value, rotation.value))
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
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
    pub fn new(
        name: String,
        frame: String,
        physical: bool,
        length: f64,
        radius: f64,
        local_transform: Isometry3<f64>,
    ) -> Self {
        // TODO: add/create shape and handle
        Self {
            name,
            frame,
            physical,
            length,
            radius,
            local_transform,
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl CapsuleShape {
    #[new]
    pub fn from_python(name:String, frame:String, physical:bool,length:f64,radius:f64,translation:PyTranslation,rotation:PyRotation) -> Self {
        Self::new(name,frame,physical,length,radius,Isometry3::from_parts(translation.value, rotation.value))
    }

    #[getter]
    pub fn get_translation(&self) -> PyResult<PyTranslation> {
        Ok(PyTranslation{value:self.local_transform.translation})
    }

    #[getter]
    pub fn get_rotation(&self) -> PyResult<PyRotation> {
        Ok(PyRotation{value:self.local_transform.rotation})
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct SphereShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub radius: f64,
    pub local_transform: Isometry3<f64>, // handle: Whatever Parry3D/Rapier handle is needed
}

impl SphereShape {
    pub fn new(
        name: String,
        frame: String,
        physical: bool,
        radius: f64,
        local_transform: Isometry3<f64>,
    ) -> Self {
        // TODO: add/create shape and handle
        Self {
            name,
            frame,
            physical,
            radius,
            local_transform,
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl SphereShape {
    #[new]
    pub fn from_python(name:String, frame:String, physical:bool,radius:f64,translation:PyTranslation,rotation:PyRotation) -> Self {
        Self::new(name,frame,physical,radius,Isometry3::from_parts(translation.value, rotation.value))
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct MeshShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub filename: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub local_transform: Isometry3<f64>,
}

impl MeshShape {
    pub fn new(
        name: String,
        frame: String,
        physical: bool,
        filename: String,
        x: f64,
        y: f64,
        z: f64,
        local_transform: Isometry3<f64>,
    ) -> Self {
        // TODO: add/create shape and handle
        Self {
            name,
            frame,
            physical,
            filename,
            x,
            y,
            z,
            local_transform,
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl MeshShape {
    #[new]
    pub fn from_python(name:String, frame:String, physical:bool,filename:String,x:f64,y:f64,z:f64,translation:PyTranslation,rotation:PyRotation) -> Self {
        Self::new(name,frame,physical,filename,x,y,z,Isometry3::from_parts(translation.value, rotation.value))
    }

    #[getter]
    pub fn get_translation(&self) -> PyResult<PyTranslation> {
        Ok(PyTranslation{value:self.local_transform.translation})
    }

    #[getter]
    pub fn get_rotation(&self) -> PyResult<PyRotation> {
        Ok(PyRotation{value:self.local_transform.rotation})
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct HullShape {
    pub frame: String,
    pub name: String,
    pub physical: bool,
    pub points: Vec<Vector3<f64>>,
    pub local_transform: Isometry3<f64>,
}

impl HullShape {
    pub fn new(
        name: String,
        frame: String,
        physical: bool,
        points: Vec<Vector3<f64>>,
        local_transform: Isometry3<f64>,
    ) -> Self {
        // TODO: add/create shape and handle
        Self {
            name,
            frame,
            physical,
            points,
            local_transform,
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl HullShape {
    #[new]
    pub fn from_python(name:String, frame:String, physical:bool,points:Vec<PyTranslation>,translation:PyTranslation,rotation:PyRotation) -> Self {
        Self::new(name,frame,physical,points.iter().map(|trans| trans.value.vector).collect(),Isometry3::from_parts(translation.value, rotation.value))
    }

    #[getter]
    pub fn get_translation(&self) -> PyResult<PyTranslation> {
        Ok(PyTranslation{value:self.local_transform.translation})
    }

    #[getter]
    pub fn get_rotation(&self) -> PyResult<PyRotation> {
        Ok(PyRotation{value:self.local_transform.rotation})
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[serde(tag = "type")]
#[cfg_attr(feature = "pybindings", derive(FromPyObject))]
pub enum Shape {
    Box(BoxShape),
    Cylinder(CylinderShape),
    Sphere(SphereShape),
    Capsule(CapsuleShape),
    Mesh(MeshShape),
    Hull(HullShape),
}

impl Shape {
    pub fn get_frame_name(self) -> String {
        match self {
            Self::Mesh(object) => {
                return object.frame;
            }
            Self::Box(object) => {
                return object.frame;
            }
            Self::Cylinder(object) => {
                return object.frame;
            }
            Self::Capsule(object) => {
                return object.frame;
            }
            Self::Hull(object) => {
                return object.frame;
            }
            Self::Sphere(object) => {
                return object.frame;
            }
        }
    }

}

#[cfg(feature = "pybindings")]
impl IntoPy<PyObject> for Shape {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Box(obj) => obj.into_py(py),
            Self::Cylinder(obj) => obj.into_py(py),
            Self::Sphere(obj) => obj.into_py(py),
            Self::Capsule(obj) => obj.into_py(py),
            Self::Mesh(obj) => obj.into_py(py),
            Self::Hull(obj) => obj.into_py(py)
        }
    }
}

// #[cfg(feature = "bevy")]
// impl Into<bevy::prelude::Mesh> for Shape {
//     fn into(self: Shape) -> bevy::prelude::Mesh {
//         match self {
//             Shape::Box(box_object) => {
//                 return bevy::prelude::Mesh::from(bevy::prelude::shape::Box::new(
//                     box_object.x as f32,
//                     box_object.y as f32,
//                     box_object.z as f32,
//                 ));
//             }
//             Shape::Cylinder(cylinder_object) => {
//                 return bevy::prelude::Mesh::from(bevy::prelude::shape::Capsule {
//                     radius: cylinder_object.radius as f32,
//                     rings: 0,
//                     depth: cylinder_object.length as f32,
//                     latitudes: 64,
//                     longitudes: 32,
//                     uv_profile: CapsuleUvProfile::Aspect,
//                 });
//             }
//             Shape::Sphere(sphere_object) => {
//                 return bevy::prelude::Mesh::from(bevy::prelude::shape::Icosphere {
//                     radius: sphere_object.radius as f32,
//                     subdivisions: 32,
//                 });
//             }
//             Shape::Capsule(capsule_object) => {
//                 return bevy::prelude::Mesh::from(bevy::prelude::shape::Capsule {
//                     radius: capsule_object.radius as f32,
//                     rings: 0,
//                     depth: capsule_object.length as f32,
//                     latitudes: 16,
//                     longitudes: 32,
//                     uv_profile: CapsuleUvProfile::Aspect,
//                 });
//             }
//             Shape::Mesh(_) => {
//                 return bevy::prelude::Mesh::from(bevy::prelude::shape::Cube { size: 1.0 });
//             }
//             Shape::Hull(_) => {
//                 return Mesh::from(bevy::prelude::shape::Cube { size: 1.0 });
//             }
//         }
//     }
// }
