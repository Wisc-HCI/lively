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
        goal: Goal::Translation
    },
    OrientationMatch {
        objective: OrientationMatchObjective,
        goal: Goal::Rotation
    },
    PositionLiveliness {
        objective: PositionLivelinessObjective,
        goal: Goal::Size
    },
    OrientationLiveliness {
        objective: OrientationLivelinessObjective,
        goal: Goal::Size
    },
    PositionMirroring {
        objective: PositionMirroringObjective,
        goal: Goal::Translation
    },
    OrientationMirroring {
        objective: OrientationMirroringObjective,
        goal: Goal::Rotation
    },
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