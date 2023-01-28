use crate::utils::shapes::{BoxShape, CapsuleShape, CylinderShape, MeshShape, Shape, SphereShape};
#[cfg(feature = "bevy")]
use bevy::sprite::collide_aabb::Collision;
use k::urdf::isometry_from;
use nalgebra::geometry::{Point3, Isometry3, UnitQuaternion};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use urdf_rs::{Geometry, Link, Mimic};
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use crate::utils::pyutils::*;

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct TransformInfo {
    #[serde(skip_deserializing, default = "Isometry3::identity")]
    pub world: Isometry3<f64>,
    #[serde(skip_deserializing, default = "Isometry3::identity")]
    pub local: Isometry3<f64>,
}

impl TransformInfo {
    pub fn new(world: Isometry3<f64>, local: Isometry3<f64>) -> Self {
        Self { world, local }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
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
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct JointInfo {
    pub name: String,
    pub joint_type: String,
    pub lower_bound: f64,
    pub upper_bound: f64,
    pub max_velocity: f64,
    pub axis: [f64; 3],
    pub mimic: Option<MimicInfo>,
    pub parent_link: String,
    pub child_link: String,

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
        parent_link: String,
        child_link: String
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
            parent_link,
            child_link
        }
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
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
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct ProximityInfo {
    pub shape1: String,
    pub shape2: String,
    pub distance: f64,
    pub points: Option<(Point3<f64>, Point3<f64>)>,
    pub physical: bool,
    pub loss: f64,
    pub average_distance: Option<f64>
}

impl ProximityInfo {
    pub fn new(
        shape1: String,
        shape2: String,
        distance: f64,
        points: Option<(Point3<f64>, Point3<f64>)>,
        physical: bool,
        loss: f64,
        average_distance: Option<f64>
    ) -> Self {
        Self {
            shape1,
            shape2,
            distance,
            points,
            physical,
            loss,
            average_distance
        }
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct CollisionSettingInfo {
    pub d_max: f64,
    pub r: f64,
    pub a_max: f64,
    pub time_budget: u64,
    pub timed: bool,
}

impl CollisionSettingInfo {
    pub fn new(d_max: f64, r: f64, a_max: f64, time_budget: u64, timed: bool) -> Self {
        Self {
            d_max,
            r,
            a_max,
            time_budget,
            timed,
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl CollisionSettingInfo {
    #[new]
    pub fn from_python(d_max: f64, r: f64, a_max: f64, time_budget: u64, timed: bool) -> Self {
        CollisionSettingInfo::new(d_max, r, a_max, time_budget, timed)
    }
}

impl Default for CollisionSettingInfo {
    fn default() -> Self {
        Self {
            d_max: 0.3,
            r: 0.0,
            a_max: 2.0,
            time_budget: 100,
            timed: true,
        }
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct AddShape {
    pub id: String,
    pub shape: Shape
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl AddShape {
    #[new]
    pub fn from_python(id: String, shape: Shape) -> Self {
        AddShape{id,shape}
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct MoveShape {
    pub id: String,
    pub transform: Isometry3<f64>
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl MoveShape {
    #[new]
    pub fn from_python(id: String, translation: PyTranslation, rotation: PyRotation) -> Self {
        MoveShape{id, transform:Isometry3::from_parts(translation.value, rotation.value)}
    }

    #[getter]
    pub fn get_translation(&self) -> PyResult<PyTranslation> {
        Ok(PyTranslation{value:self.transform.translation})
    }

    #[getter]
    pub fn get_rotation(&self) -> PyResult<PyRotation> {
        Ok(PyRotation{value:self.transform.rotation})
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
#[cfg_attr(feature = "pybindings", derive(FromPyObject))]
pub enum ShapeUpdate {
    Add(AddShape),
    Move(MoveShape),
    Delete(String),
}

#[cfg(feature = "pybindings")]
impl IntoPy<PyObject> for ShapeUpdate {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Add(obj) => obj.into_py(py),
            Self::Move(obj) => obj.into_py(py),
            Self::Delete(obj) => obj.into_py(py)
        }
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default,Copy)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct ScalarRange {
    pub value: f64,
    pub delta: f64
}

impl ScalarRange {
    pub fn new(value:f64, delta:f64) -> Self {
        Self {value,delta}
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl ScalarRange {
    #[new]
    pub fn from_python(value: f64, delta: f64) -> Self {
        Self::new(value,delta)
    }
}


#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default,Copy)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct RotationRange {
    pub rotation: UnitQuaternion<f64>,
    pub delta: f64
}

impl RotationRange {
    pub fn new(rotation:UnitQuaternion<f64>, delta:f64) -> Self {
        Self {rotation,delta}
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl RotationRange {
    #[new]
    pub fn from_python(rotation: PyRotation, delta: f64) -> Self {
        Self::new(rotation.value,delta)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default,Copy)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct Ellipse {
    pub transform: Isometry3<f64>,
    pub size: Vector3<f64>
}

impl Ellipse {
    pub fn new(transform:Isometry3<f64>, size:Vector3<f64>) -> Self {
        Self {transform,size}
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl Ellipse {
    #[new]
    pub fn from_python(translation: PyTranslation, rotation: PyRotation, size: PySize) -> Self {
        Self::new(Isometry3::from_parts(translation.value,rotation.value),size.value)
    }
}