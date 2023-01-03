use crate::objectives::core::base::*;
use crate::objectives::core::bounding::*;
use crate::objectives::core::matching::*;
use crate::objectives::core::mirroring::*;
use crate::objectives::liveliness::forces::*;
use crate::objectives::liveliness::perlin::*;
use crate::utils::goals::Goal;
use nalgebra::geometry::UnitQuaternion;
use nalgebra::Vector3;

pub enum BehaviorProperty {
    PositionMatch {
        objective: PositionMatchObjective,
        goal: Option::Some(Goal::Translation)
    },
    OrientationMatch {
        objective: OrientationMatchObjective,
        goal: Option::Some(Goal::Rotation)
    },
    PositionLiveliness {
        objective: PositionLivelinessObjective,
        goal: Option::Some(Goal::Size)
    },
    OrientationLiveliness {
        objective: OrientationLivelinessObjective,
        goal: Option::Some(Goal::Size)
    },
    PositionMirroring {
        objective: PositionMirroringObjective,
        goal: Option::Some(Goal::Translation)
    },
    OrientationMirroring {
        objective: OrientationMirroringObjective,
        goal: Option::Some(Goal::Rotation)
    },
    PositionBounding {
        objective: PositionBoundingObjective,
        goal: Option::Some(Goal::Ellipse)
    },
    OrientationBounding{
        objective: OrientationBoundingObjective,
        goal: Option::Some(Goal::RotationRange)
    },
    JointMatch{
        objective: JointMatchObjective,
        goal: Option::Some(Goal::ScalarRange)
    },
    JointLiveliness{
        objective: JointLivelinessObjective,
        goal: Option::Some(Goal::Scalar)
    },
    JointMirroring{
        objective: JointMirroringObjective,
        goal: Option::Some(Goal::Scalar)
    },
    JointLimits{
        objective: JointLimitsObjective,
        goal: None
    },
    JointBounding{
        objective: JointBoundingObjective,
        goal: None
    },
    CollisionAvoidance{
        objective: CollisionAvoidanceObjective,
        goal: None
    },
    VelocityMinimization{
        objective: VelocityMinimizationObjective,
        goal: None
    },
    AccelerationMinimization{
        objective: AccelerationMinimizationObjective,
        goal: None
    },
    JerkMinimization{
        objective: JerkMinimizationObjective,
        goal: None
    },
    OriginVelocityMinimization{
        objective: OriginVelocityMinimizationObjective,
        goal: None
    },
    OriginAccelerationMinimization{
        objective: OriginAccelerationMinimizationObjective,
        goal: None
    },
    OriginJerkMinimization{
        objective: OriginJerkMinimizationObjective,
        goal: None
    },
    RelativeMotionLiveliness{
        objective: RelativeMotionLivelinessObjective,
        goal: Option::Some(Goal::Scalar)
    },
    Gravity{
        objective: GravityObjective,
        goal: None
    },
    SmoothnessMacro{
        objective: SmoothnessMacroObjective,
        goal: None
    },
    DistanceMatch{
        objective: DistanceMatchObjective,
        goal: Option::Some(Goal::Scalar)
    }
}