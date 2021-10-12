use pyo3::prelude::*;
use crate::utils::geometry::{*};

#[derive(Clone,Debug,FromPyObject)]
pub enum Goal {
    Translation(Translation),
    Rotation(Rotation),
    Scalar(f64),
    Size(Size),
    Ellipse(Ellipse),
    RotationRange(RotationRange),
    ScalarRange(ScalarRange)
}

impl IntoPy<PyObject> for Goal {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Translation(obj) => obj.into_py(py),
            Self::Rotation(obj) => obj.into_py(py),
            Self::Scalar(obj) => obj.into_py(py),
            Self::Size(obj) => obj.into_py(py),
            Self::Ellipse(obj) => obj.into_py(py),
            Self::RotationRange(obj) => obj.into_py(py),
            Self::ScalarRange(obj) => obj.into_py(py),
        }
    }
}