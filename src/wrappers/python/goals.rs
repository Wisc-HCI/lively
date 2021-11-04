#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use crate::utils::goals::Goal;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::geometry::{*};
#[cfg(feature = "pybindings")]
use nalgebra::geometry::{Isometry3};

#[cfg(feature = "pybindings")]
#[derive(Clone,Debug,FromPyObject)]
pub enum PyGoal {
    Translation(Translation),
    Rotation(Rotation),
    Scalar(f64),
    Size(Size),
    Ellipse(Ellipse),
    RotationRange(RotationRange),
    ScalarRange(ScalarRange)
}

#[cfg(feature = "pybindings")]
impl IntoPy<PyObject> for PyGoal {
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

#[cfg(feature = "pybindings")]
impl From<PyGoal> for Goal {
    fn from(pygoal: PyGoal) -> Goal {
        match pygoal {
            PyGoal::Translation(translation) => return Goal::Translation(translation.value),
            PyGoal::Rotation(rotation) => return Goal::Rotation(rotation.value),
            PyGoal::Scalar(value) => return Goal::Scalar(value),
            PyGoal::Size(size) => return Goal::Size(size.value),
            PyGoal::Ellipse(ellipse) => return Goal::Ellipse{pose:Isometry3::from_parts(ellipse.translation.value,ellipse.rotation.value),size:ellipse.size.value},
            PyGoal::RotationRange(rotation_range) => return Goal::RotationRange{rotation:rotation_range.rotation.value,delta:rotation_range.delta},
            PyGoal::ScalarRange(scalar_range) => return Goal::ScalarRange{value:scalar_range.value,delta:scalar_range.delta}
        }
    }
}

#[cfg(feature = "pybindings")]
impl From<Goal> for PyGoal {
    fn from(goal: Goal) -> PyGoal {
        match goal {
            Goal::Translation(translation) => return PyGoal::Translation(Translation{value:translation}),
            Goal::Rotation(rotation) => return PyGoal::Rotation(Rotation{value:rotation}),
            Goal::Scalar(value) => return PyGoal::Scalar(value),
            Goal::Size(size) => return PyGoal::Size(Size{value:size}),
            Goal::Ellipse{pose,size} => return PyGoal::Ellipse(Ellipse{translation: Translation{value:pose.translation}, rotation: Rotation{value:pose.rotation}, size: Size{value:size}}),
            Goal::RotationRange{rotation,delta} => return PyGoal::RotationRange(RotationRange{rotation: Rotation{value:rotation},delta}),
            Goal::ScalarRange{value,delta} => return PyGoal::ScalarRange(ScalarRange{value,delta})
        }
    }
}