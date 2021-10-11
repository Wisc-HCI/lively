use pyo3::prelude::*;
// use nalgebra::geometry::{Isometry3<f64>};
// use nalgebra::{Vector3};
use crate::utils::state::State;
use crate::objectives::core::base::{*};
use crate::objectives::core::bounding::{*};
use crate::objectives::core::matching::{*};
use crate::objectives::core::mirroring::{*};
use crate::objectives::liveliness::forces::{*};
use crate::objectives::liveliness::perlin::{*};


#[derive(Clone,Debug,FromPyObject)]
pub enum Objective {
    PositionMatch(PositionMatchObjective),
    OrientationMatch(OrientationMatchObjective),
    PositionLiveliness(PositionLivelinessObjective),
    OrientationLiveliness(OrientationLivelinessObjective),
    PositionMirroring(PositionMirroringObjective),
    OrientationMirroring(OrientationMirroringObjective),
    PositionBounding(PositionBoundingObjective),
    OrientationBounding(OrientationBoundingObjective),
    JointMatch(JointMatchObjective),
    JointLiveliness(JointLivelinessObjective),
    JointMirroring(JointMirroringObjective),
    JointLimits(JointLimitsObjective),
    CollisionAvoidance(CollisionAvoidanceObjective),
    MinimizeVelocity(MinimizeVelocityObjective),
    MinimizeAcceleration(MinimizeAccelerationObjective),
    MinimizeJerk(MinimizeJerkObjective),
    RelativeMotionLiveliness(RelativeMotionLivelinessObjective),
    OriginPositionLiveliness(OriginPositionLivelinessObjective),
    OriginOrientationLiveliness(OriginOrientationLivelinessObjective),
    OriginPositionMatch(OriginPositionMatchObjective),
    OriginOrientationMatch(OriginOrientationMatchObjective),
    Gravity(GravityObjective),
    MacroSmoothness(MacroSmoothness),
    DistanceMatch(DistanceMatchObjective)
}

impl IntoPy<PyObject> for Objective {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::PositionMatch(obj) => obj.into_py(py),
            Self::OrientationMatch(obj) => obj.into_py(py)
            Self::PositionLiveliness(obj) => obj.into_py(py),
            Self::OrientationLiveliness(obj) => obj.into_py(py),
            Self::PositionMirroring(obj) => obj.into_py(py),
            Self::OrientationMirroring(obj) => obj.into_py(py),
            Self::PositionBounding(obj) => obj.into_py(py),
            Self::OrientationBounding(obj) => obj.into_py(py),
            Self::JointMatch(obj) => obj.into_py(py),
            Self::JointLiveliness(obj) => obj.into_py(py),
            Self::JointMirroring(obj) => obj.into_py(py),
            Self::JointLimits(obj) => obj.into_py(py),
            Self::JointBounding(obj) => obj.into_py(py),
            Self::CollisionAvoidance(obj) => obj.into_py(py),
            Self::MinimizeVelocity(obj) => obj.into_py(py),
            Self::MinimizeAcceleration(obj) => obj.into_py(py),
            Self::MinimizeJerk(obj) => obj.into_py(py),
            Self::RelativeMotionLiveliness(obj) => obj.into_py(py),
            Self::OriginPositionLiveliness(obj) => obj.into_py(py),
            Self::OriginOrientationLiveliness(obj) => obj.into_py(py),
            Self::OriginPositionMatch(obj) => obj.into_py(py),
            Self::OriginOrientationMatch(obj) => obj.into_py(py),
            Self::Gravity(obj) => obj.into_py(py),
            Self::MacroSmoothness(obj) => obj.into_py(py),
            Self::DistanceMatch(obj) => obj.into_py(py),
        }
    }
}

impl Objective {
    pub fn call(
                &self,
                v: &Vars,
                state: &State,
                is_core: bool,
            ) -> f64 {
                match self {
                    Self::PositionMatch(obj) => obj.call(v,state,is_core),
                    Self::OrientationMatch(obj) => obj.call(v,state,is_core)
                    Self::PositionLiveliness(obj) => obj.call(v,state,is_core),
                    Self::OrientationLiveliness(obj) => obj.call(v,state,is_core),
                    Self::PositionMirroring(obj) => obj.call(v,state,is_core),
                    Self::OrientationMirroring(obj) => obj.call(v,state,is_core),
                    Self::PositionBounding(obj) => obj.call(v,state,is_core),
                    Self::OrientationBounding(obj) => obj.call(v,state,is_core),
                    Self::JointMatch(obj) => obj.call(v,state,is_core),
                    Self::JointLiveliness(obj) => obj.call(v,state,is_core),
                    Self::JointMirroring(obj) => obj.call(v,state,is_core),
                    Self::JointLimits(obj) => obj.call(v,state,is_core),
                    Self::JointBounding(obj) => obj.call(v,state,is_core),
                    Self::CollisionAvoidance(obj) => obj.call(v,state,is_core),
                    Self::MinimizeVelocity(obj) => obj.call(v,state,is_core),
                    Self::MinimizeAcceleration(obj) => obj.call(v,state,is_core),
                    Self::MinimizeJerk(obj) => obj.call(v,state,is_core),
                    Self::RelativeMotionLiveliness(obj) => obj.call(v,state,is_core),
                    Self::OriginPositionLiveliness(obj) => obj.call(v,state,is_core),
                    Self::OriginOrientationLiveliness(obj) => obj.call(v,state,is_core),
                    Self::OriginPositionMatch(obj) => obj.call(v,state,is_core),
                    Self::OriginOrientationMatch(obj) => obj.call(v,state,is_core),
                    Self::Gravity(obj) => obj.call(v,state,is_core),
                    Self::MacroSmoothness(obj) => obj.call(v,state,is_core),
                    Self::DistanceMatch(obj) => obj.call(v,state,is_core)
                }
    }

    pub fn update(&mut self, time: f64) {
        match self {
            Self::PositionLiveliness(obj) => obj.update(time),
            Self::OrientationLiveliness(obj) => obj.update(time),
            Self::JointLiveliness(obj) => obj.update(time),
            Self::RelativeMotionLiveliness(obj) => obj.update(time),
            Self::OriginPositionLiveliness(obj) => obj.update(time),
            Self::OriginOrientationLiveliness(obj) => obj.update(time),
            _ => println!("Tried updating an objective that doesn't support it {}",self)
        }
    }

    pub fn set_weight(&mut self, weight: f64) {
        match self {
            Self::PositionMatch(obj) => obj.weight = weight,
            Self::OrientationMatch(obj) => obj.weight = weight
            Self::PositionLiveliness(obj) => obj.weight = weight,
            Self::OrientationLiveliness(obj) => obj.weight = weight,
            Self::PositionMirroring(obj) => obj.weight = weight,
            Self::OrientationMirroring(obj) => obj.weight = weight,
            Self::PositionBounding(obj) => obj.weight = weight,
            Self::OrientationBounding(obj) => obj.weight = weight,
            Self::JointMatch(obj) => obj.weight = weight,
            Self::JointLiveliness(obj) => obj.weight = weight,
            Self::JointMirroring(obj) => obj.weight = weight,
            Self::JointLimits(obj) => obj.weight = weight,
            Self::JointBounding(obj) => obj.weight = weight,
            Self::CollisionAvoidance(obj) => obj.weight = weight,
            Self::MinimizeVelocity(obj) => obj.weight = weight,
            Self::MinimizeAcceleration(obj) => obj.weight = weight,
            Self::MinimizeJerk(obj) => obj.weight = weight,
            Self::RelativeMotionLiveliness(obj) => obj.weight = weight,
            Self::OriginPositionLiveliness(obj) => obj.weight = weight,
            Self::OriginOrientationLiveliness(obj) => obj.weight = weight,
            Self::OriginPositionMatch(obj) => obj.weight = weight,
            Self::OriginOrientationMatch(obj) => obj.weight = weight,
            Self::Gravity(obj) => obj.weight = weight,
            Self::MacroSmoothness(obj) => obj.weight = weight,
            Self::DistanceMatch(obj) => obj.weight = weight
        }
    }

    pub fn set_goal(&mut self, goal: Goal) {
        match goal {
            Goal::Translation(translation_goal) => {
                match self {
                    Self::PositionMatch(obj) => obj.goal = translation_goal.value.vector,
                    Self::PositionMirroring(obj) => obj.goal = translation_goal.value.vector,
                    Self::OriginPositionMatch(obj) => obj.goal = translation_goal.value.vector,
                    _ => println!("Unexpected Goal type provided {} for Objective {}",goal,self)
                }
            },
            Goal::Rotation(rotation_goal) => {
                match self {
                    Self::OrientationMatch(obj) => obj.goal = translation_goal.value,
                    Self::OrientationMirroring(obj) => obj.goal = translation_goal.value,
                    Self::OriginOrientationMatch(obj) => obj.goal = translation_goal.value,
                    _ => println!("Unexpected Goal type provided {} for Objective {}",goal,self)
                }
            },
            Goal::Scalar(scalar_goal) => {
                match self {
                    Self::JointMatch(obj) => obj.goal = scalar_goal,
                    Self::JointMirroring(obj) => obj.goal = scalar_goal,
                    Self::DistanceMatch(obj) => obj.goal = scalar_goal,
                    Self::JointLiveliness(obj) => obj.goal = goal_goal,
                    Self::RelativeMotionLiveliness(obj) => obj.goal = goal_goal,
                    _ => println!("Unexpected Goal type provided {} for Objective {}",goal,self)
                }
            },
            Goal::Size(size_goal) => {
                match self {
                    Self::PositionLiveliness(obj) => obj.goal = size_goal.value,
                    Self::OrientationLiveliness(obj) => obj.goal = size_goal.value,
                    Self::OriginPositionLiveliness(obj) => obj.goal = size_goal.value,
                    Self::OriginOrientationLiveliness(obj) => obj.goal = size_goal.value,
                    _ => println!("Unexpected Goal type provided {} for Objective {}",goal,self)
                }
            },
            Goal::Ellipse(translation,rotation,size) => {
                match self {
                    Self::PositionBounding(obj) => obj.goal = (translation.value.vector,rotation.value,size.value),
                    _ => println!("Unexpected Goal type provided {} for Objective {}",goal,self)
                }
            },
            Goal::RotationRange(rotation,delta) => {
                match self {
                    Self::OrientationBounding(obj) => obj.goal = (rotation.value,delta),
                    _ => println!("Unexpected Goal type provided {} for Objective {}",goal,self)
                }
            },
            Goal::ScalarRange(value,delta) => {
                match self {
                    Self::JointBounding(obj) => obj.goal = (value,delta),
                    _ => println!("Unexpected Goal type provided {} for Objective {}",goal,self)
                }
            }
            Goal::None => {
                println!("Unexpected Goal type provided {} for Objective {}",goal,self)
            }
        }
    }
}

pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -((-(x_val - t).powi(d)) / (2.0 * c.powi(2))).exp() + f * (x_val - t).powi(g)
}

pub fn groove_loss_derivative(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -((-(x_val - t).powi(d)) / (2.0 * c.powi(2))).exp()
        * ((-d as f64 * (x_val - t)) / (2.0 * c.powi(2)))
        + g as f64 * f * (x_val - t)
}
