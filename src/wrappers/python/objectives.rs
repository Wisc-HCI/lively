
use pyo3::prelude::*;
use crate::objectives::objective::Objective;
use crate::objectives::core::base::*;
use crate::objectives::core::bounding::*;
use crate::objectives::core::matching::*;
use crate::objectives::core::mirroring::*;
use crate::objectives::liveliness::forces::*;
use crate::objectives::liveliness::perlin::*;

#[derive(Clone,Debug,FromPyObject)]
pub enum PyObjective {
	PositionMatch(PyPositionMatchObjective),
	OrientationMatch(PyOrientationMatchObjective),
	PositionLiveliness(PyPositionLivelinessObjective),
	OrientationLiveliness(PyOrientationLivelinessObjective),
	PositionMirroring(PyPositionMirroringObjective),
	OrientationMirroring(PyOrientationMirroringObjective),
	PositionBounding(PyPositionBoundingObjective),
	OrientationBounding(PyOrientationBoundingObjective),
	JointMatch(PyJointMatchObjective),
	JointLiveliness(PyJointLivelinessObjective),
	JointMirroring(PyJointMirroringObjective),
	JointBounding(PyJointBoundingObjective),
	JointLimits(PyJointLimitsObjective),
	CollisionAvoidance(PyCollisionAvoidanceObjective),
	VelocityMinimization(PyVelocityMinimizationObjective),
	AccelerationMinimization(PyAccelerationMinimizationObjective),
	JerkMinimization(PyJerkMinimizationObjective),
	OriginVelocityMinimization(PyOriginVelocityMinimizationObjective),
	OriginAccelerationMinimization(PyOriginAccelerationMinimizationObjective),
	OriginJerkMinimization(PyOriginJerkMinimizationObjective),
	RelativeMotionLiveliness(PyRelativeMotionLivelinessObjective),
	OriginPositionLiveliness(PyOriginPositionLivelinessObjective),
	OriginOrientationLiveliness(PyOriginOrientationLivelinessObjective),
	OriginPositionMatch(PyOriginPositionMatchObjective),
	OriginOrientationMatch(PyOriginOrientationMatchObjective),
	Gravity(PyGravityObjective),
	SmoothnessMacro(PySmoothnessMacroObjective),
	DistanceMatch(PyDistanceMatchObjective),
}

impl IntoPy<PyObject> for PyObjective {
    fn into_py(self, py: Python) -> PyObject {
        match self {
			Self::PositionMatch(obj) => obj.into_py(py),
			Self::OrientationMatch(obj) => obj.into_py(py),
			Self::PositionLiveliness(obj) => obj.into_py(py),
			Self::OrientationLiveliness(obj) => obj.into_py(py),
			Self::PositionMirroring(obj) => obj.into_py(py),
			Self::OrientationMirroring(obj) => obj.into_py(py),
			Self::PositionBounding(obj) => obj.into_py(py),
			Self::OrientationBounding(obj) => obj.into_py(py),
			Self::JointMatch(obj) => obj.into_py(py),
			Self::JointLiveliness(obj) => obj.into_py(py),
			Self::JointMirroring(obj) => obj.into_py(py),
			Self::JointBounding(obj) => obj.into_py(py),
			Self::JointLimits(obj) => obj.into_py(py),
			Self::CollisionAvoidance(obj) => obj.into_py(py),
			Self::VelocityMinimization(obj) => obj.into_py(py),
			Self::AccelerationMinimization(obj) => obj.into_py(py),
			Self::JerkMinimization(obj) => obj.into_py(py),
			Self::OriginVelocityMinimization(obj) => obj.into_py(py),
			Self::OriginAccelerationMinimization(obj) => obj.into_py(py),
			Self::OriginJerkMinimization(obj) => obj.into_py(py),
			Self::RelativeMotionLiveliness(obj) => obj.into_py(py),
			Self::OriginPositionLiveliness(obj) => obj.into_py(py),
			Self::OriginOrientationLiveliness(obj) => obj.into_py(py),
			Self::OriginPositionMatch(obj) => obj.into_py(py),
			Self::OriginOrientationMatch(obj) => obj.into_py(py),
			Self::Gravity(obj) => obj.into_py(py),
			Self::SmoothnessMacro(obj) => obj.into_py(py),
			Self::DistanceMatch(obj) => obj.into_py(py),
        }
    }
}

impl From<Objective> for PyObjective {
    fn from(objective:Objective) -> PyObjective {
        match objective {
			Objective::PositionMatch(obj) => Self::PositionMatch(PyPositionMatchObjective(obj)),
			Objective::OrientationMatch(obj) => Self::OrientationMatch(PyOrientationMatchObjective(obj)),
			Objective::PositionLiveliness(obj) => Self::PositionLiveliness(PyPositionLivelinessObjective(obj)),
			Objective::OrientationLiveliness(obj) => Self::OrientationLiveliness(PyOrientationLivelinessObjective(obj)),
			Objective::PositionMirroring(obj) => Self::PositionMirroring(PyPositionMirroringObjective(obj)),
			Objective::OrientationMirroring(obj) => Self::OrientationMirroring(PyOrientationMirroringObjective(obj)),
			Objective::PositionBounding(obj) => Self::PositionBounding(PyPositionBoundingObjective(obj)),
			Objective::OrientationBounding(obj) => Self::OrientationBounding(PyOrientationBoundingObjective(obj)),
			Objective::JointMatch(obj) => Self::JointMatch(PyJointMatchObjective(obj)),
			Objective::JointLiveliness(obj) => Self::JointLiveliness(PyJointLivelinessObjective(obj)),
			Objective::JointMirroring(obj) => Self::JointMirroring(PyJointMirroringObjective(obj)),
			Objective::JointBounding(obj) => Self::JointBounding(PyJointBoundingObjective(obj)),
			Objective::JointLimits(obj) => Self::JointLimits(PyJointLimitsObjective(obj)),
			Objective::CollisionAvoidance(obj) => Self::CollisionAvoidance(PyCollisionAvoidanceObjective(obj)),
			Objective::VelocityMinimization(obj) => Self::VelocityMinimization(PyVelocityMinimizationObjective(obj)),
			Objective::AccelerationMinimization(obj) => Self::AccelerationMinimization(PyAccelerationMinimizationObjective(obj)),
			Objective::JerkMinimization(obj) => Self::JerkMinimization(PyJerkMinimizationObjective(obj)),
			Objective::OriginVelocityMinimization(obj) => Self::OriginVelocityMinimization(PyOriginVelocityMinimizationObjective(obj)),
			Objective::OriginAccelerationMinimization(obj) => Self::OriginAccelerationMinimization(PyOriginAccelerationMinimizationObjective(obj)),
			Objective::OriginJerkMinimization(obj) => Self::OriginJerkMinimization(PyOriginJerkMinimizationObjective(obj)),
			Objective::RelativeMotionLiveliness(obj) => Self::RelativeMotionLiveliness(PyRelativeMotionLivelinessObjective(obj)),
			Objective::OriginPositionLiveliness(obj) => Self::OriginPositionLiveliness(PyOriginPositionLivelinessObjective(obj)),
			Objective::OriginOrientationLiveliness(obj) => Self::OriginOrientationLiveliness(PyOriginOrientationLivelinessObjective(obj)),
			Objective::OriginPositionMatch(obj) => Self::OriginPositionMatch(PyOriginPositionMatchObjective(obj)),
			Objective::OriginOrientationMatch(obj) => Self::OriginOrientationMatch(PyOriginOrientationMatchObjective(obj)),
			Objective::Gravity(obj) => Self::Gravity(PyGravityObjective(obj)),
			Objective::SmoothnessMacro(obj) => Self::SmoothnessMacro(PySmoothnessMacroObjective(obj)),
			Objective::DistanceMatch(obj) => Self::DistanceMatch(PyDistanceMatchObjective(obj)),
        }
    }
}

impl From<PyObjective> for Objective {
    fn from(pyobjective:PyObjective) -> Objective {
        match pyobjective {
			PyObjective::PositionMatch(obj) => Self::PositionMatch(obj.0),
			PyObjective::OrientationMatch(obj) => Self::OrientationMatch(obj.0),
			PyObjective::PositionLiveliness(obj) => Self::PositionLiveliness(obj.0),
			PyObjective::OrientationLiveliness(obj) => Self::OrientationLiveliness(obj.0),
			PyObjective::PositionMirroring(obj) => Self::PositionMirroring(obj.0),
			PyObjective::OrientationMirroring(obj) => Self::OrientationMirroring(obj.0),
			PyObjective::PositionBounding(obj) => Self::PositionBounding(obj.0),
			PyObjective::OrientationBounding(obj) => Self::OrientationBounding(obj.0),
			PyObjective::JointMatch(obj) => Self::JointMatch(obj.0),
			PyObjective::JointLiveliness(obj) => Self::JointLiveliness(obj.0),
			PyObjective::JointMirroring(obj) => Self::JointMirroring(obj.0),
			PyObjective::JointBounding(obj) => Self::JointBounding(obj.0),
			PyObjective::JointLimits(obj) => Self::JointLimits(obj.0),
			PyObjective::CollisionAvoidance(obj) => Self::CollisionAvoidance(obj.0),
			PyObjective::VelocityMinimization(obj) => Self::VelocityMinimization(obj.0),
			PyObjective::AccelerationMinimization(obj) => Self::AccelerationMinimization(obj.0),
			PyObjective::JerkMinimization(obj) => Self::JerkMinimization(obj.0),
			PyObjective::OriginVelocityMinimization(obj) => Self::OriginVelocityMinimization(obj.0),
			PyObjective::OriginAccelerationMinimization(obj) => Self::OriginAccelerationMinimization(obj.0),
			PyObjective::OriginJerkMinimization(obj) => Self::OriginJerkMinimization(obj.0),
			PyObjective::RelativeMotionLiveliness(obj) => Self::RelativeMotionLiveliness(obj.0),
			PyObjective::OriginPositionLiveliness(obj) => Self::OriginPositionLiveliness(obj.0),
			PyObjective::OriginOrientationLiveliness(obj) => Self::OriginOrientationLiveliness(obj.0),
			PyObjective::OriginPositionMatch(obj) => Self::OriginPositionMatch(obj.0),
			PyObjective::OriginOrientationMatch(obj) => Self::OriginOrientationMatch(obj.0),
			PyObjective::Gravity(obj) => Self::Gravity(obj.0),
			PyObjective::SmoothnessMacro(obj) => Self::SmoothnessMacro(obj.0),
			PyObjective::DistanceMatch(obj) => Self::DistanceMatch(obj.0),
        }
    }
}


#[pyclass(name="PositionMatchObjective")]
#[derive(Clone,Debug)]
pub struct PyPositionMatchObjective(PositionMatchObjective);

#[pymethods]
impl PyPositionMatchObjective {
    #[new]
    pub fn new(name:String,weight:f64,link:String) -> Self {
        PyPositionMatchObjective(PositionMatchObjective::new(name,weight,link))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.0.link.clone())
    }

}


#[pyclass(name="OrientationMatchObjective")]
#[derive(Clone,Debug)]
pub struct PyOrientationMatchObjective(OrientationMatchObjective);

#[pymethods]
impl PyOrientationMatchObjective {
    #[new]
    pub fn new(name:String,weight:f64,link:String) -> Self {
        PyOrientationMatchObjective(OrientationMatchObjective::new(name,weight,link))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.0.link.clone())
    }

}


#[pyclass(name="PositionLivelinessObjective")]
#[derive(Clone,Debug)]
pub struct PyPositionLivelinessObjective(PositionLivelinessObjective);

#[pymethods]
impl PyPositionLivelinessObjective {
    #[new]
    pub fn new(name:String,weight:f64,link:String,frequency:f64) -> Self {
        PyPositionLivelinessObjective(PositionLivelinessObjective::new(name,weight,link,frequency))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.0.link.clone())
    }

    #[getter]
    pub fn get_frequency(&self) -> PyResult<f64> {
        Ok(self.0.frequency.clone())
    }

}


#[pyclass(name="OrientationLivelinessObjective")]
#[derive(Clone,Debug)]
pub struct PyOrientationLivelinessObjective(OrientationLivelinessObjective);

#[pymethods]
impl PyOrientationLivelinessObjective {
    #[new]
    pub fn new(name:String,weight:f64,link:String,frequency:f64) -> Self {
        PyOrientationLivelinessObjective(OrientationLivelinessObjective::new(name,weight,link,frequency))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.0.link.clone())
    }

    #[getter]
    pub fn get_frequency(&self) -> PyResult<f64> {
        Ok(self.0.frequency.clone())
    }

}


#[pyclass(name="PositionMirroringObjective")]
#[derive(Clone,Debug)]
pub struct PyPositionMirroringObjective(PositionMirroringObjective);

#[pymethods]
impl PyPositionMirroringObjective {
    #[new]
    pub fn new(name:String,weight:f64,link1:String,link2:String) -> Self {
        PyPositionMirroringObjective(PositionMirroringObjective::new(name,weight,link1,link2))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link1(&self) -> PyResult<String> {
        Ok(self.0.link1.clone())
    }

    #[getter]
    pub fn get_link2(&self) -> PyResult<String> {
        Ok(self.0.link2.clone())
    }

}


#[pyclass(name="OrientationMirroringObjective")]
#[derive(Clone,Debug)]
pub struct PyOrientationMirroringObjective(OrientationMirroringObjective);

#[pymethods]
impl PyOrientationMirroringObjective {
    #[new]
    pub fn new(name:String,weight:f64,link1:String,link2:String) -> Self {
        PyOrientationMirroringObjective(OrientationMirroringObjective::new(name,weight,link1,link2))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link1(&self) -> PyResult<String> {
        Ok(self.0.link1.clone())
    }

    #[getter]
    pub fn get_link2(&self) -> PyResult<String> {
        Ok(self.0.link2.clone())
    }

}


#[pyclass(name="PositionBoundingObjective")]
#[derive(Clone,Debug)]
pub struct PyPositionBoundingObjective(PositionBoundingObjective);

#[pymethods]
impl PyPositionBoundingObjective {
    #[new]
    pub fn new(name:String,weight:f64,link:String) -> Self {
        PyPositionBoundingObjective(PositionBoundingObjective::new(name,weight,link))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.0.link.clone())
    }

}


#[pyclass(name="OrientationBoundingObjective")]
#[derive(Clone,Debug)]
pub struct PyOrientationBoundingObjective(OrientationBoundingObjective);

#[pymethods]
impl PyOrientationBoundingObjective {
    #[new]
    pub fn new(name:String,weight:f64,link:String) -> Self {
        PyOrientationBoundingObjective(OrientationBoundingObjective::new(name,weight,link))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.0.link.clone())
    }

}


#[pyclass(name="JointMatchObjective")]
#[derive(Clone,Debug)]
pub struct PyJointMatchObjective(JointMatchObjective);

#[pymethods]
impl PyJointMatchObjective {
    #[new]
    pub fn new(name:String,weight:f64,joint:String) -> Self {
        PyJointMatchObjective(JointMatchObjective::new(name,weight,joint))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_joint(&self) -> PyResult<String> {
        Ok(self.0.joint.clone())
    }

}


#[pyclass(name="JointLivelinessObjective")]
#[derive(Clone,Debug)]
pub struct PyJointLivelinessObjective(JointLivelinessObjective);

#[pymethods]
impl PyJointLivelinessObjective {
    #[new]
    pub fn new(name:String,weight:f64,joint:String,frequency:f64) -> Self {
        PyJointLivelinessObjective(JointLivelinessObjective::new(name,weight,joint,frequency))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_joint(&self) -> PyResult<String> {
        Ok(self.0.joint.clone())
    }

    #[getter]
    pub fn get_frequency(&self) -> PyResult<f64> {
        Ok(self.0.frequency.clone())
    }

}


#[pyclass(name="JointMirroringObjective")]
#[derive(Clone,Debug)]
pub struct PyJointMirroringObjective(JointMirroringObjective);

#[pymethods]
impl PyJointMirroringObjective {
    #[new]
    pub fn new(name:String,weight:f64,joint1:String,joint2:String) -> Self {
        PyJointMirroringObjective(JointMirroringObjective::new(name,weight,joint1,joint2))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_joint1(&self) -> PyResult<String> {
        Ok(self.0.joint1.clone())
    }

    #[getter]
    pub fn get_joint2(&self) -> PyResult<String> {
        Ok(self.0.joint2.clone())
    }

}


#[pyclass(name="JointBoundingObjective")]
#[derive(Clone,Debug)]
pub struct PyJointBoundingObjective(JointBoundingObjective);

#[pymethods]
impl PyJointBoundingObjective {
    #[new]
    pub fn new(name:String,weight:f64,joint:String) -> Self {
        PyJointBoundingObjective(JointBoundingObjective::new(name,weight,joint))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_joint(&self) -> PyResult<String> {
        Ok(self.0.joint.clone())
    }

}


#[pyclass(name="JointLimitsObjective")]
#[derive(Clone,Debug)]
pub struct PyJointLimitsObjective(JointLimitsObjective);

#[pymethods]
impl PyJointLimitsObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyJointLimitsObjective(JointLimitsObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="CollisionAvoidanceObjective")]
#[derive(Clone,Debug)]
pub struct PyCollisionAvoidanceObjective(CollisionAvoidanceObjective);

#[pymethods]
impl PyCollisionAvoidanceObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyCollisionAvoidanceObjective(CollisionAvoidanceObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="VelocityMinimizationObjective")]
#[derive(Clone,Debug)]
pub struct PyVelocityMinimizationObjective(VelocityMinimizationObjective);

#[pymethods]
impl PyVelocityMinimizationObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyVelocityMinimizationObjective(VelocityMinimizationObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="AccelerationMinimizationObjective")]
#[derive(Clone,Debug)]
pub struct PyAccelerationMinimizationObjective(AccelerationMinimizationObjective);

#[pymethods]
impl PyAccelerationMinimizationObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyAccelerationMinimizationObjective(AccelerationMinimizationObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="JerkMinimizationObjective")]
#[derive(Clone,Debug)]
pub struct PyJerkMinimizationObjective(JerkMinimizationObjective);

#[pymethods]
impl PyJerkMinimizationObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyJerkMinimizationObjective(JerkMinimizationObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="OriginVelocityMinimizationObjective")]
#[derive(Clone,Debug)]
pub struct PyOriginVelocityMinimizationObjective(OriginVelocityMinimizationObjective);

#[pymethods]
impl PyOriginVelocityMinimizationObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyOriginVelocityMinimizationObjective(OriginVelocityMinimizationObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="OriginAccelerationMinimizationObjective")]
#[derive(Clone,Debug)]
pub struct PyOriginAccelerationMinimizationObjective(OriginAccelerationMinimizationObjective);

#[pymethods]
impl PyOriginAccelerationMinimizationObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyOriginAccelerationMinimizationObjective(OriginAccelerationMinimizationObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="OriginJerkMinimizationObjective")]
#[derive(Clone,Debug)]
pub struct PyOriginJerkMinimizationObjective(OriginJerkMinimizationObjective);

#[pymethods]
impl PyOriginJerkMinimizationObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyOriginJerkMinimizationObjective(OriginJerkMinimizationObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="RelativeMotionLivelinessObjective")]
#[derive(Clone,Debug)]
pub struct PyRelativeMotionLivelinessObjective(RelativeMotionLivelinessObjective);

#[pymethods]
impl PyRelativeMotionLivelinessObjective {
    #[new]
    pub fn new(name:String,weight:f64,link1:String,link2:String,frequency:f64) -> Self {
        PyRelativeMotionLivelinessObjective(RelativeMotionLivelinessObjective::new(name,weight,link1,link2,frequency))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link1(&self) -> PyResult<String> {
        Ok(self.0.link1.clone())
    }

    #[getter]
    pub fn get_link2(&self) -> PyResult<String> {
        Ok(self.0.link2.clone())
    }

    #[getter]
    pub fn get_frequency(&self) -> PyResult<f64> {
        Ok(self.0.frequency.clone())
    }

}


#[pyclass(name="OriginPositionLivelinessObjective")]
#[derive(Clone,Debug)]
pub struct PyOriginPositionLivelinessObjective(OriginPositionLivelinessObjective);

#[pymethods]
impl PyOriginPositionLivelinessObjective {
    #[new]
    pub fn new(name:String,weight:f64,frequency:f64) -> Self {
        PyOriginPositionLivelinessObjective(OriginPositionLivelinessObjective::new(name,weight,frequency))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_frequency(&self) -> PyResult<f64> {
        Ok(self.0.frequency.clone())
    }

}


#[pyclass(name="OriginOrientationLivelinessObjective")]
#[derive(Clone,Debug)]
pub struct PyOriginOrientationLivelinessObjective(OriginOrientationLivelinessObjective);

#[pymethods]
impl PyOriginOrientationLivelinessObjective {
    #[new]
    pub fn new(name:String,weight:f64,frequency:f64) -> Self {
        PyOriginOrientationLivelinessObjective(OriginOrientationLivelinessObjective::new(name,weight,frequency))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_frequency(&self) -> PyResult<f64> {
        Ok(self.0.frequency.clone())
    }

}


#[pyclass(name="OriginPositionMatchObjective")]
#[derive(Clone,Debug)]
pub struct PyOriginPositionMatchObjective(OriginPositionMatchObjective);

#[pymethods]
impl PyOriginPositionMatchObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyOriginPositionMatchObjective(OriginPositionMatchObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="OriginOrientationMatchObjective")]
#[derive(Clone,Debug)]
pub struct PyOriginOrientationMatchObjective(OriginOrientationMatchObjective);

#[pymethods]
impl PyOriginOrientationMatchObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PyOriginOrientationMatchObjective(OriginOrientationMatchObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="GravityObjective")]
#[derive(Clone,Debug)]
pub struct PyGravityObjective(GravityObjective);

#[pymethods]
impl PyGravityObjective {
    #[new]
    pub fn new(name:String,weight:f64,link:String) -> Self {
        PyGravityObjective(GravityObjective::new(name,weight,link))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.0.link.clone())
    }

}


#[pyclass(name="SmoothnessMacroObjective")]
#[derive(Clone,Debug)]
pub struct PySmoothnessMacroObjective(SmoothnessMacroObjective);

#[pymethods]
impl PySmoothnessMacroObjective {
    #[new]
    pub fn new(name:String,weight:f64) -> Self {
        PySmoothnessMacroObjective(SmoothnessMacroObjective::new(name,weight))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

}


#[pyclass(name="DistanceMatchObjective")]
#[derive(Clone,Debug)]
pub struct PyDistanceMatchObjective(DistanceMatchObjective);

#[pymethods]
impl PyDistanceMatchObjective {
    #[new]
    pub fn new(name:String,weight:f64,link1:String,link2:String) -> Self {
        PyDistanceMatchObjective(DistanceMatchObjective::new(name,weight,link1,link2))
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.0.weight.clone())
    }

    #[getter]
    pub fn get_link1(&self) -> PyResult<String> {
        Ok(self.0.link1.clone())
    }

    #[getter]
    pub fn get_link2(&self) -> PyResult<String> {
        Ok(self.0.link2.clone())
    }

}
