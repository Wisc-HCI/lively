use serde::{Serialize,Deserialize};
use nalgebra::geometry::{Translation3, Isometry3, UnitQuaternion};
use nalgebra::{Vector3};

#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(tag = "type")]
pub enum Goal {
    Translation(Translation3<f64>),
    Rotation(UnitQuaternion<f64>),
    Scalar(f64),
    Size(Vector3<f64>),
    Ellipse{
        pose: Isometry3<f64>,
        size: Vector3<f64>
    },
    RotationRange{
        rotation: UnitQuaternion<f64>,
        delta: f64
    },
    ScalarRange{
        value: f64,
        delta: f64
    }
}