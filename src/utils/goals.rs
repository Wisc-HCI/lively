use pyo3::prelude::*;
use crate::utils::geometry::{*};

#[derive(Clone,Debug,FromPyObject)]
pub enum Goal {
    Translation(Translation),
    Rotation(Rotation),
    Scalar(f64),
    Size(Size),
    Ellipse(Translation,Rotation,Size),
    RotationRange(Rotation,f64),
    ScalarRange(f64,f64),
    None
}

impl IntoPy<PyObject> for Goal {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Translation(obj) => obj.into_py(py),
            Self::Rotation(obj) => obj.into_py(py),
            Self::Scalar(obj) => obj.into_py(py),
            Self::Size(obj) => obj.into_py(py),
            Self::Ellipse(translation,rotation,size) => (translation.into_py(py),rotation.into_py(py),size.into_py(py)),
            Self::RotationRange(rotation,delta) => (rotation.into_py(py),delta.into_py(py)),
            Self::ScalarRange(value,delta) => (value.into_py(py),delta.into_py(py)),
            Self::None => None.into_py(py)
        }
    }
}