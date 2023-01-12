use serde::{Serialize,Deserialize};
use crate::utils::state::State;
use crate::utils::vars::Vars;
use crate::utils::goals::Goal;
use crate::objectives::core::base::{*};
use crate::objectives::core::bounding::{*};
use crate::objectives::core::matching::{*};
use crate::objectives::core::mirroring::{*};
use crate::objectives::liveliness::forces::{*};
use crate::objectives::liveliness::perlin::{*};
use nalgebra::Translation3;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(tag = "type")]
#[cfg_attr(feature = "pybindings", derive(FromPyObject))]
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
    JointBounding(JointBoundingObjective),
    CollisionAvoidance(CollisionAvoidanceObjective),
    JointVelocityMinimization(JointVelocityMinimizationObjective),
    JointAccelerationMinimization(JointAccelerationMinimizationObjective),
    JointJerkMinimization(JointJerkMinimizationObjective),
    OriginVelocityMinimization(OriginVelocityMinimizationObjective),
    OriginAccelerationMinimization(OriginAccelerationMinimizationObjective),
    OriginJerkMinimization(OriginJerkMinimizationObjective),
    LinkVelocityMinimization(LinkVelocityMinimizationObjective),
    LinkAccelerationMinimization(LinkAccelerationMinimizationObjective),
    LinkJerkMinimization(LinkJerkMinimizationObjective),
    RelativeMotionLiveliness(RelativeMotionLivelinessObjective),
    Gravity(GravityObjective),
    SmoothnessMacro(SmoothnessMacroObjective),
    DistanceMatch(DistanceMatchObjective)
}

impl Objective {
    pub fn get_type(&self) -> String {
        // Returns a string value for each variant. Useful in debugging.
        match self {
            Self::PositionMatch(_obj) => return String::from("PositionMatchObjective"),
            Self::OrientationMatch(_obj) => return String::from("OrientationnMatchObjective"),
            Self::PositionLiveliness(_obj) => return String::from("PositionLivelinessObjective"),
            Self::OrientationLiveliness(_obj) => return String::from("OrientationLivelinessObjective"),
            Self::PositionMirroring(_obj) => return String::from("PositionMirroringObjective"),
            Self::OrientationMirroring(_obj) => return String::from("OrientationMirroringObjective"),
            Self::PositionBounding(_obj) => return String::from("PositionBoundingObjective"),
            Self::OrientationBounding(_obj) => return String::from("OrientationBoundingObjective"),
            Self::JointMatch(_obj) => return String::from("JointMatchObjective"),
            Self::JointLiveliness(_obj) => return String::from("JointLivelinessObjective"),
            Self::JointMirroring(_obj) => return String::from("JointMirroringObjective"),
            Self::JointLimits(_obj) => return String::from("JointLimitsObjective"),
            Self::JointBounding(_obj) => return String::from("JointBoundingObjective"),
            Self::CollisionAvoidance(_obj) => return String::from("CollisionAvoidanceObjective"),
            Self::JointVelocityMinimization(_obj) => return String::from("JointVelocityMinimizationObjective"),
            Self::JointAccelerationMinimization(_obj) => return String::from("JointAccelerationMinimizationObjective"),
            Self::JointJerkMinimization(_obj) => return String::from("JointJerkMinimizationObjective"),
            Self::OriginVelocityMinimization(_obj) => return String::from("OriginVelocityMinimizationObjective"),
            Self::OriginAccelerationMinimization(_obj) => return String::from("OriginAccelerationMinimizationObjective"),
            Self::OriginJerkMinimization(_obj) => return String::from("OriginJerkMinimizationObjective"),
            Self::LinkVelocityMinimization(_obj) => return String::from("LinkVelocityMinimizationObjective"),
            Self::LinkAccelerationMinimization(_obj) => return String::from("LinkAccelerationMinimizationObjective"),
            Self::LinkJerkMinimization(_obj) => return String::from("LinkJerkMinimizationObjective"),
            Self::RelativeMotionLiveliness(_obj) => return String::from("RelativeMotionLivelinessObjective"),
            Self::Gravity(_obj) => return String::from("GravityObjective"),
            Self::SmoothnessMacro(_obj) => return String::from("SmoothnessMacroObjective"),
            Self::DistanceMatch(_obj) => return String::from("DistanceMatchObjective"),
        }
    }

    pub fn call(
                &self,
                v: &Vars,
                state: &State
            ) -> f64 {
                // A switch that passes along the `call` method to the inner objective.
                match self {
                    Self::PositionMatch(obj) => obj.call(v,state),
                    Self::OrientationMatch(obj) => obj.call(v,state),
                    Self::PositionLiveliness(obj) => obj.call(v,state),
                    Self::OrientationLiveliness(obj) => obj.call(v,state),
                    Self::PositionMirroring(obj) => obj.call(v,state),
                    Self::OrientationMirroring(obj) => obj.call(v,state),
                    Self::PositionBounding(obj) => obj.call(v,state),
                    Self::OrientationBounding(obj) => obj.call(v,state),
                    Self::JointMatch(obj) => obj.call(v,state),
                    Self::JointLiveliness(obj) => obj.call(v,state),
                    Self::JointMirroring(obj) => obj.call(v,state),
                    Self::JointLimits(obj) => obj.call(v,state),
                    Self::JointBounding(obj) => obj.call(v,state),
                    Self::CollisionAvoidance(obj) => obj.call(v,state),
                    Self::JointVelocityMinimization(obj) => obj.call(v,state),
                    Self::JointAccelerationMinimization(obj) => obj.call(v,state),
                    Self::JointJerkMinimization(obj) => obj.call(v,state),
                    Self::OriginVelocityMinimization(obj) => obj.call(v,state),
                    Self::OriginAccelerationMinimization(obj) => obj.call(v,state),
                    Self::OriginJerkMinimization(obj) => obj.call(v,state),
                    Self::LinkVelocityMinimization(obj) => obj.call(v,state),
                    Self::LinkAccelerationMinimization(obj) => obj.call(v,state),
                    Self::LinkJerkMinimization(obj) => obj.call(v,state),
                    Self::RelativeMotionLiveliness(obj) => obj.call(v,state),
                    Self::Gravity(obj) => obj.call(v,state),
                    Self::SmoothnessMacro(obj) => obj.call(v,state),
                    Self::DistanceMatch(obj) => obj.call(v,state)
                }
    }

    pub fn update(&mut self, time: f64) {
        // For time-sensitive objectives, include them here.
        match self {
            Self::PositionLiveliness(obj) => obj.update(time),
            Self::OrientationLiveliness(obj) => obj.update(time),
            Self::JointLiveliness(obj) => obj.update(time),
            Self::RelativeMotionLiveliness(obj) => obj.update(time),
            _ => {}
        }
    }

    pub fn set_weight(&mut self, weight: f64) {
        // Set the weight for the inner objective
        match self {
            Self::PositionMatch(obj) => obj.set_weight(weight),
            Self::OrientationMatch(obj) => obj.set_weight(weight),
            Self::PositionLiveliness(obj) => obj.set_weight(weight),
            Self::OrientationLiveliness(obj) => obj.set_weight(weight),
            Self::PositionMirroring(obj) => obj.set_weight(weight),
            Self::OrientationMirroring(obj) => obj.set_weight(weight),
            Self::PositionBounding(obj) => obj.set_weight(weight),
            Self::OrientationBounding(obj) => obj.set_weight(weight),
            Self::JointMatch(obj) => obj.set_weight(weight),
            Self::JointLiveliness(obj) => obj.set_weight(weight),
            Self::JointMirroring(obj) => obj.set_weight(weight),
            Self::JointLimits(obj) => obj.set_weight(weight),
            Self::JointBounding(obj) => obj.set_weight(weight),
            Self::CollisionAvoidance(obj) => obj.set_weight(weight),
            Self::JointVelocityMinimization(obj) => obj.set_weight(weight),
            Self::JointAccelerationMinimization(obj) => obj.set_weight(weight),
            Self::JointJerkMinimization(obj) => obj.set_weight(weight),
            Self::OriginVelocityMinimization(obj) => obj.set_weight(weight),
            Self::OriginAccelerationMinimization(obj) => obj.set_weight(weight),
            Self::OriginJerkMinimization(obj) => obj.set_weight(weight),
            Self::LinkVelocityMinimization(obj) => obj.set_weight(weight),
            Self::LinkAccelerationMinimization(obj) => obj.set_weight(weight),
            Self::LinkJerkMinimization(obj) => obj.set_weight(weight),
            Self::RelativeMotionLiveliness(obj) => obj.set_weight(weight),
            Self::Gravity(obj) => obj.set_weight(weight),
            Self::SmoothnessMacro(obj) => obj.set_weight(weight),
            Self::DistanceMatch(obj) =>  obj.set_weight(weight)
        }
    }

    pub fn get_goal(&self) -> Option<Goal> {
        // get the goal for the inner objective. Useful for debugging.
        match self {
            Self::PositionMatch(obj) => return Some(Goal::Translation(Translation3::from(obj.goal))),
            Self::OrientationMatch(obj) => return Some(Goal::Rotation(obj.goal)),
            Self::PositionLiveliness(obj) => return Some(Goal::Size(obj.goal)),
            Self::OrientationLiveliness(obj) => return Some(Goal::Size(obj.goal)),
            Self::PositionMirroring(obj) => return Some(Goal::Translation(Translation3::from(obj.goal))),
            Self::OrientationMirroring(obj) => return Some(Goal::Rotation(obj.goal)),
            Self::PositionBounding(obj) => return Some(Goal::Ellipse(obj.goal)),
            Self::OrientationBounding(obj) => return Some(Goal::RotationRange(obj.goal)),
            Self::JointMatch(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::JointLiveliness(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::JointMirroring(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::JointLimits(_obj) => return None,
            Self::JointBounding(obj) => return Some(Goal::ScalarRange(obj.goal)),
            Self::CollisionAvoidance(_obj) => return None,
            Self::JointVelocityMinimization(_obj) => return None,
            Self::JointAccelerationMinimization(_obj) => return None,
            Self::JointJerkMinimization(_obj) => return None,
            Self::OriginVelocityMinimization(_obj) => return None,
            Self::OriginAccelerationMinimization(_obj) => return None,
            Self::OriginJerkMinimization(_obj) => return None,
            Self::LinkVelocityMinimization(_obj) => return None,
            Self::LinkAccelerationMinimization(_obj) => return None,
            Self::LinkJerkMinimization(_obj) => return None,
            Self::RelativeMotionLiveliness(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::Gravity(_obj) => return None,
            Self::SmoothnessMacro(_obj) => return None,
            Self::DistanceMatch(obj) => return Some(Goal::Scalar(obj.goal))
        }
    }

    pub fn set_goal(&mut self, goal: &Goal) {
        // Set the goal for the inner objective. This matches based on Objective and Goal variant. 
        match (goal,self) {
            (Goal::Translation(translation_goal),Self::PositionMatch(obj)) => obj.set_goal(translation_goal.vector),
            (Goal::Translation(translation_goal),Self::PositionMirroring(obj)) => obj.set_goal(translation_goal.vector),
            (Goal::Rotation(rotation_goal),Self::OrientationMatch(obj)) => obj.set_goal(*rotation_goal),
            (Goal::Rotation(rotation_goal),Self::OrientationMirroring(obj)) => obj.set_goal(*rotation_goal),
            (Goal::Scalar(scalar_goal),Self::JointMatch(obj)) => obj.set_goal(*scalar_goal),
            (Goal::Scalar(scalar_goal),Self::JointMirroring(obj)) => obj.set_goal(*scalar_goal),
            (Goal::Scalar(scalar_goal),Self::DistanceMatch(obj)) => obj.set_goal(*scalar_goal),
            (Goal::Scalar(scalar_goal),Self::JointLiveliness(obj)) => obj.set_goal(*scalar_goal),
            (Goal::Scalar(scalar_goal),Self::RelativeMotionLiveliness(obj)) => obj.set_goal(*scalar_goal),
            (Goal::Size(size_goal),Self::PositionLiveliness(obj)) => obj.set_goal(*size_goal),
            (Goal::Size(size_goal),Self::OrientationLiveliness(obj)) => obj.set_goal(*size_goal),
            (Goal::Ellipse(ellipse_goal),Self::PositionBounding(obj)) => obj.set_goal(*ellipse_goal),
            (Goal::RotationRange(rotation_range_goal),Self::OrientationBounding(obj)) => obj.set_goal(*rotation_range_goal),
            (Goal::ScalarRange(scalar_range_goal),Self::JointBounding(obj)) => obj.set_goal(*scalar_range_goal),
            (g,o) => {
                println!("Unexpected goal {:?} provided for Objective {:?}",g,o.clone())
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

pub trait Callable<T> {
    fn update(&mut self, _time: f64) {}

    fn set_goal(&mut self, _goal: T) {}

    fn set_weight(&mut self, weight: f64);

    fn call(&self,
        v: &Vars,
        state: &State) -> f64;
}

#[cfg(feature = "pybindings")]
impl IntoPy<PyObject> for Objective {
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
			Self::JointVelocityMinimization(obj) => obj.into_py(py),
			Self::JointAccelerationMinimization(obj) => obj.into_py(py),
			Self::JointJerkMinimization(obj) => obj.into_py(py),
			Self::OriginVelocityMinimization(obj) => obj.into_py(py),
			Self::OriginAccelerationMinimization(obj) => obj.into_py(py),
			Self::OriginJerkMinimization(obj) => obj.into_py(py),
            Self::LinkVelocityMinimization(obj) => obj.into_py(py),
			Self::LinkAccelerationMinimization(obj) => obj.into_py(py),
			Self::LinkJerkMinimization(obj) => obj.into_py(py),
			Self::RelativeMotionLiveliness(obj) => obj.into_py(py),
			Self::Gravity(obj) => obj.into_py(py),
			Self::SmoothnessMacro(obj) => obj.into_py(py),
			Self::DistanceMatch(obj) => obj.into_py(py),
        }
    }
}