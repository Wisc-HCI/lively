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
    OriginPositionLiveliness(OriginPositionLivelinessObjective),
    OriginOrientationLiveliness(OriginOrientationLivelinessObjective),
    OriginPositionMatch(OriginPositionMatchObjective),
    OriginOrientationMatch(OriginOrientationMatchObjective),
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
            Self::OriginPositionLiveliness(_obj) => return String::from("OriginPositionLivelinessObjective"),
            Self::OriginOrientationLiveliness(_obj) => return String::from("OriginOrientationLivelinessObjective"),
            Self::OriginPositionMatch(_obj) => return String::from("OriginPositionMatchObjective"),
            Self::OriginOrientationMatch(_obj) => return String::from("OriginOrientationMatchObjective"),
            Self::Gravity(_obj) => return String::from("GravityObjective"),
            Self::SmoothnessMacro(_obj) => return String::from("SmoothnessMacroObjective"),
            Self::DistanceMatch(_obj) => return String::from("DistanceMatchObjective"),
        }
    }

    pub fn call(
                &self,
                v: &Vars,
                state: &State,
                is_core: bool,
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
                    Self::OriginPositionLiveliness(obj) => obj.call(v,state,is_core),
                    Self::OriginOrientationLiveliness(obj) => obj.call(v,state,is_core),
                    Self::OriginPositionMatch(obj) => obj.call(v,state,is_core),
                    Self::OriginOrientationMatch(obj) => obj.call(v,state,is_core),
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
            Self::OriginPositionLiveliness(obj) => obj.update(time),
            Self::OriginOrientationLiveliness(obj) => obj.update(time),
            _ => {}
        }
    }

    pub fn set_weight(&mut self, weight: f64) {
        match self {
            Self::PositionMatch(obj) => obj.weight = weight,
            Self::OrientationMatch(obj) => obj.weight = weight,
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
            Self::VelocityMinimization(obj) => obj.weight = weight,
            Self::AccelerationMinimization(obj) => obj.weight = weight,
            Self::JerkMinimization(obj) => obj.weight = weight,
            Self::OriginVelocityMinimization(obj) => obj.weight = weight,
            Self::OriginAccelerationMinimization(obj) => obj.weight = weight,
            Self::OriginJerkMinimization(obj) => obj.weight = weight,
            Self::RelativeMotionLiveliness(obj) => obj.weight = weight,
            Self::OriginPositionLiveliness(obj) => obj.weight = weight,
            Self::OriginOrientationLiveliness(obj) => obj.weight = weight,
            Self::OriginPositionMatch(obj) => obj.weight = weight,
            Self::OriginOrientationMatch(obj) => obj.weight = weight,
            Self::Gravity(obj) => obj.weight = weight,
            Self::SmoothnessMacro(obj) => obj.weight = weight,
            Self::DistanceMatch(obj) => obj.weight = weight
        }
    }

    pub fn set_goal(&mut self, goal: &Goal) {
        match goal {
            Goal::Translation(translation_goal) => {
                match self {
                    Self::PositionMatch(obj) => obj.goal = translation_goal.vector,
                    Self::PositionMirroring(obj) => obj.goal = translation_goal.vector,
                    Self::OriginPositionMatch(obj) => obj.goal = translation_goal.vector,
                    _ => println!("Unexpected Translation provided for Objective {}",self.get_type())
                }
            },
            Goal::Rotation(rotation_goal) => {
                match self {
                    Self::OrientationMatch(obj) => obj.goal = *rotation_goal,
                    Self::OrientationMirroring(obj) => obj.goal = *rotation_goal,
                    Self::OriginOrientationMatch(obj) => obj.goal = *rotation_goal,
                    _ => println!("Unexpected Rotation Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::Scalar(scalar_goal) => {
                match self {
                    Self::JointMatch(obj) => obj.goal = *scalar_goal,
                    Self::JointMirroring(obj) => obj.goal = *scalar_goal,
                    Self::DistanceMatch(obj) => obj.goal = *scalar_goal,
                    Self::JointLiveliness(obj) => obj.goal = *scalar_goal,
                    Self::RelativeMotionLiveliness(obj) => obj.goal = *scalar_goal,
                    _ => println!("Unexpected Scalar Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::Size(size_goal) => {
                match self {
                    Self::PositionLiveliness(obj) => obj.goal = *size_goal,
                    Self::OrientationLiveliness(obj) => obj.goal = *size_goal,
                    Self::OriginPositionLiveliness(obj) => obj.goal = *size_goal,
                    Self::OriginOrientationLiveliness(obj) => obj.goal = *size_goal,
                    _ => println!("Unexpected Size Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::Ellipse{pose,size} => {
                match self {
                    Self::PositionBounding(obj) => obj.goal = (*pose,*size),
                    _ => println!("Unexpected Ellipse Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::RotationRange{rotation,delta} => {
                match self {
                    Self::OrientationBounding(obj) => obj.goal = (*rotation,*delta),
                    _ => println!("Unexpected Rotation Goal type provided for Objective {}",self.get_type())
                }
            },
            Goal::ScalarRange{value,delta} => {
                match self {
                    Self::JointBounding(obj) => obj.goal = (*value,*delta),
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

