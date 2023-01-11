use serde::{Serialize,Deserialize};
use nalgebra::geometry::{Translation3, UnitQuaternion};
use nalgebra::Vector3;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;

use crate::utils::info::{ScalarRange,RotationRange,Ellipse};
#[cfg(feature = "pybindings")]
use crate::utils::pyutils::{PyTranslation,PyRotation,PySize};


#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum Goal {
    Translation(Translation3<f64>),
    Rotation(UnitQuaternion<f64>),
    Scalar(f64),
    Size(Vector3<f64>),
    Ellipse(Ellipse),
    RotationRange(RotationRange),
    ScalarRange(ScalarRange)
}

#[cfg(feature = "pybindings")]
impl IntoPy<PyObject> for Goal {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Translation(obj) => PyTranslation{value:obj}.into_py(py),
            Self::Rotation(obj) => PyRotation{value:obj}.into_py(py),
            Self::Scalar(obj) => obj.into_py(py),
            Self::Size(obj) => PySize{value:obj}.into_py(py),
            Self::Ellipse(obj) => obj.into_py(py),
            Self::RotationRange(obj) => obj.into_py(py),
            Self::ScalarRange(obj) => obj.into_py(py),
        }
    }
}

#[cfg(feature = "pybindings")]
impl FromPyObject<'_> for Goal {
    fn extract(ob: &'_ PyAny) -> PyResult<Self> {

        if let Ok(ob) = f64::extract(ob) {
            return Ok(Self::Scalar(ob))
        }

        if let Ok(ob) = PyTranslation::extract(ob) {
            return Ok(Self::Translation(ob.value))
        } 

        if let Ok(ob) = PySize::extract(ob) {
            return Ok(Self::Size(ob.value))
        }

        if let Ok(ob) = PyRotation::extract(ob) {
            return Ok(Self::Rotation(ob.value))
        }

        if let Ok(ob) = Ellipse::extract(ob) {
            return Ok(Self::Ellipse(ob))
        }

        if let Ok(ob) = RotationRange::extract(ob) {
            return Ok(Self::RotationRange(ob))
        }

        if let Ok(ob) = ScalarRange::extract(ob) {
            return Ok(Self::ScalarRange(ob))
        }

        return Ok(Self::Scalar(0.0));
    }
}