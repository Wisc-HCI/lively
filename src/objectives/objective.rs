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

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(tag = "type")]
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
    VelocityMinimization(VelocityMinimizationObjective),
    AccelerationMinimization(AccelerationMinimizationObjective),
    JerkMinimization(JerkMinimizationObjective),
    OriginVelocityMinimization(OriginVelocityMinimizationObjective),
    OriginAccelerationMinimization(OriginAccelerationMinimizationObjective),
    OriginJerkMinimization(OriginJerkMinimizationObjective),
    RelativeMotionLiveliness(RelativeMotionLivelinessObjective),
    Gravity(GravityObjective),
    SmoothnessMacro(SmoothnessMacroObjective),
    DistanceMatch(DistanceMatchObjective)
}

impl Objective {
    pub fn get_type(&self) -> String {
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
            Self::VelocityMinimization(_obj) => return String::from("VelocityMinimizationObjective"),
            Self::AccelerationMinimization(_obj) => return String::from("AccelerationMinimizationObjective"),
            Self::JerkMinimization(_obj) => return String::from("JerkMinimizationObjective"),
            Self::OriginVelocityMinimization(_obj) => return String::from("OriginVelocityMinimizationObjective"),
            Self::OriginAccelerationMinimization(_obj) => return String::from("OriginAccelerationMinimizationObjective"),
            Self::OriginJerkMinimization(_obj) => return String::from("OriginJerkMinimizationObjective"),
            Self::RelativeMotionLiveliness(_obj) => return String::from("RelativeMotionLivelinessObjective"),
            Self::Gravity(_obj) => return String::from("GravityObjective"),
            Self::SmoothnessMacro(_obj) => return String::from("SmoothnessMacroObjective"),
            Self::DistanceMatch(_obj) => return String::from("DistanceMatchObjective"),
        }
    }

    pub fn call(
                &self,
                v: &Vars,
                state: &State,
                is_core: bool
            ) -> f64 {
                match self {
                    Self::PositionMatch(obj) => obj.call(v,state,is_core),
                    Self::OrientationMatch(obj) => obj.call(v,state,is_core),
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
                    Self::VelocityMinimization(obj) => obj.call(v,state,is_core),
                    Self::AccelerationMinimization(obj) => obj.call(v,state,is_core),
                    Self::JerkMinimization(obj) => obj.call(v,state,is_core),
                    Self::OriginVelocityMinimization(obj) => obj.call(v,state,is_core),
                    Self::OriginAccelerationMinimization(obj) => obj.call(v,state,is_core),
                    Self::OriginJerkMinimization(obj) => obj.call(v,state,is_core),
                    Self::RelativeMotionLiveliness(obj) => obj.call(v,state,is_core),
                    Self::Gravity(obj) => obj.call(v,state,is_core),
                    Self::SmoothnessMacro(obj) => obj.call(v,state,is_core),
                    Self::DistanceMatch(obj) => obj.call(v,state,is_core)
                }
    }

    pub fn update(&mut self, time: f64) {
        match self {
            Self::PositionLiveliness(obj) => obj.update(time),
            Self::OrientationLiveliness(obj) => obj.update(time),
            Self::JointLiveliness(obj) => obj.update(time),
            Self::RelativeMotionLiveliness(obj) => obj.update(time),
            _ => {}
        }
    }

    pub fn set_weight(&mut self, weight: f64) {
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
            Self::VelocityMinimization(obj) => obj.set_weight(weight),
            Self::AccelerationMinimization(obj) => obj.set_weight(weight),
            Self::JerkMinimization(obj) => obj.set_weight(weight),
            Self::OriginVelocityMinimization(obj) => obj.set_weight(weight),
            Self::OriginAccelerationMinimization(obj) => obj.set_weight(weight),
            Self::OriginJerkMinimization(obj) => obj.set_weight(weight),
            Self::RelativeMotionLiveliness(obj) => obj.set_weight(weight),
            Self::Gravity(obj) => obj.set_weight(weight),
            Self::SmoothnessMacro(obj) => obj.set_weight(weight),
            Self::DistanceMatch(obj) =>  obj.set_weight(weight)
        }
    }

    pub fn get_goal(&self) -> Option<Goal> {
        match self {
            Self::PositionMatch(obj) => return Some(Goal::Translation(Translation3::from(obj.goal))),
            Self::OrientationMatch(obj) => return Some(Goal::Rotation(obj.goal)),
            Self::PositionLiveliness(obj) => return Some(Goal::Size(obj.goal)),
            Self::OrientationLiveliness(obj) => return Some(Goal::Size(obj.goal)),
            Self::PositionMirroring(obj) => return Some(Goal::Translation(Translation3::from(obj.goal))),
            Self::OrientationMirroring(obj) => return Some(Goal::Rotation(obj.goal)),
            Self::PositionBounding(obj) => return Some(Goal::Ellipse {pose: obj.goal.0, size: obj.goal.1 }),
            Self::OrientationBounding(obj) => return Some(Goal::RotationRange {rotation: obj.goal.0, delta: obj.goal.1}),
            Self::JointMatch(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::JointLiveliness(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::JointMirroring(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::JointLimits(_obj) => return None,
            Self::JointBounding(obj) => return Some(Goal::ScalarRange {value: obj.goal.0, delta: obj.goal.1}),
            Self::CollisionAvoidance(_obj) => return None,
            Self::VelocityMinimization(_obj) => return None,
            Self::AccelerationMinimization(_obj) => return None,
            Self::JerkMinimization(_obj) => return None,
            Self::OriginVelocityMinimization(_obj) => return None,
            Self::OriginAccelerationMinimization(_obj) => return None,
            Self::OriginJerkMinimization(_obj) => return None,
            Self::RelativeMotionLiveliness(obj) => return Some(Goal::Scalar(obj.goal)),
            Self::Gravity(_obj) => return None,
            Self::SmoothnessMacro(_obj) => return None,
            Self::DistanceMatch(obj) => return Some(Goal::Scalar(obj.goal))
        }
    }

    pub fn set_goal(&mut self, goal: &Goal) {
        match goal {
            Goal::Translation(translation_goal) => {
                match self {
                    Self::PositionMatch(obj) => obj.set_goal(translation_goal.vector),
                    Self::PositionMirroring(obj) => obj.set_goal(translation_goal.vector),
                    _ => println!("Unexpected Translation provided for Objective {}",self.get_type())
                }
            },
            Goal::Rotation(rotation_goal) => {
                match self {
                    Self::OrientationMatch(obj) => obj.set_goal(*rotation_goal),
                    Self::OrientationMirroring(obj) => obj.set_goal(*rotation_goal),
                    _ => println!("Unexpected Rotation Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::Scalar(scalar_goal) => {
                match self {
                    Self::JointMatch(obj) => obj.set_goal(*scalar_goal),
                    Self::JointMirroring(obj) => obj.set_goal(*scalar_goal),
                    Self::DistanceMatch(obj) => obj.set_goal(*scalar_goal),
                    Self::JointLiveliness(obj) => obj.set_goal(*scalar_goal),
                    Self::RelativeMotionLiveliness(obj) => obj.set_goal(*scalar_goal),
                    _ => println!("Unexpected Scalar Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::Size(size_goal) => {
                match self {
                    Self::PositionLiveliness(obj) => obj.set_goal(*size_goal),
                    Self::OrientationLiveliness(obj) => obj.set_goal(*size_goal),
                    _ => println!("Unexpected Size Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::Ellipse{pose,size} => {
                match self {
                    Self::PositionBounding(obj) => obj.set_goal((*pose,*size)),
                    _ => println!("Unexpected Ellipse Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::RotationRange{rotation,delta} => {
                match self {
                    Self::OrientationBounding(obj) => obj.set_goal((*rotation,*delta)),
                    _ => println!("Unexpected Rotation Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::ScalarRange{value,delta} => {
                match self {
                    Self::JointBounding(obj) => obj.set_goal((*value,*delta)),
                    _ => println!("Unexpected Scalar Goal type provided for Objective {}",self.get_type())
                }
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
        state: &State,
        is_core: bool) -> f64;
}