extern crate pyo3;
use pyo3::prelude::*;

pub mod utils;
pub mod objectives;
pub mod lively_tk;
pub mod wrappers;

use crate::wrappers::python::solver::PySolver;
use crate::wrappers::python::geometry::{*};
use crate::wrappers::python::objectives::{*};
use crate::wrappers::python::shapes::{*};
use crate::wrappers::python::state::PyState;
use crate::wrappers::python::info::{*};

#[pymodule]
fn lively_tk(_py: Python, m: &PyModule) -> PyResult<()> {
    // State
    m.add_class::<PyState>()?;
    // Info
    m.add_class::<PyMimicInfo>()?;
    m.add_class::<PyJointInfo>()?;
    m.add_class::<PyLinkInfo>()?;
    m.add_class::<PyProximityInfo>()?;
    // Shapes
    m.add_class::<PyBoxShape>()?;
    m.add_class::<PySphereShape>()?;
    m.add_class::<PyCylinderShape>()?;
    m.add_class::<PyCapsuleShape>()?;
    m.add_class::<PyBoxZone>()?;
    m.add_class::<PySphereZone>()?;
    m.add_class::<PyCylinderZone>()?;
    m.add_class::<PyCapsuleZone>()?;
    // Geometry/Goals
    m.add_class::<Size>()?;
    m.add_class::<Translation>()?;
    m.add_class::<Rotation>()?;
    m.add_class::<Transform>()?;
    m.add_class::<Ellipse>()?;
    m.add_class::<RotationRange>()?;
    m.add_class::<ScalarRange>()?;
    // Objectives
    m.add_class::<PyPositionMatchObjective>()?;
    m.add_class::<PyOrientationMatchObjective>()?;
    m.add_class::<PyPositionLivelinessObjective>()?;
    m.add_class::<PyOrientationLivelinessObjective>()?;
    m.add_class::<PyPositionMirroringObjective>()?;
    m.add_class::<PyOrientationMirroringObjective>()?;
    m.add_class::<PyPositionBoundingObjective>()?;
    m.add_class::<PyOrientationBoundingObjective>()?;
    m.add_class::<PyJointBoundingObjective>()?;
    m.add_class::<PyJointMatchObjective>()?;
    m.add_class::<PyJointLivelinessObjective>()?;
    m.add_class::<PyJointMirroringObjective>()?;
    m.add_class::<PyJointLimitsObjective>()?;
    m.add_class::<PyCollisionAvoidanceObjective>()?;
    m.add_class::<PyVelocityMinimizationObjective>()?;
    m.add_class::<PyAccelerationMinimizationObjective>()?;
    m.add_class::<PyJerkMinimizationObjective>()?;
    m.add_class::<PyOriginVelocityMinimizationObjective>()?;
    m.add_class::<PyOriginAccelerationMinimizationObjective>()?;
    m.add_class::<PyOriginJerkMinimizationObjective>()?;
    m.add_class::<PyRelativeMotionLivelinessObjective>()?;
    m.add_class::<PyOriginPositionLivelinessObjective>()?;
    m.add_class::<PyOriginPositionMatchObjective>()?;
    m.add_class::<PyGravityObjective>()?;
    m.add_class::<PySmoothnessMacroObjective>()?;
    m.add_class::<PyDistanceMatchObjective>()?;
    // Solver
    m.add_class::<PySolver>()?;
    Ok(())
}
