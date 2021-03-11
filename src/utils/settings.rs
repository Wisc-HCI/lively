#[derive(Clone,Debug,PartialEq)]
pub enum ControlMode {
    Relative, // Control is relative to initial
    Absolute, // Control is absolute (relative to world)
}

#[derive(Clone,Debug,PartialEq)]
pub enum EnvironmentMode {
    ECA,  // (Default): incorporates environment collision avoidance
    ECA3, // Incorporates environment collision avoidance, but only considers end-effector position goal matching,
          // and does not try to match the orientation of goals
    ECAA, // Incorporates environment collision avoidance, adaptively adjusts the weight on the orientation
          // matching objectives on-the-fly, such that the importance of orientationmatching is reduced
          // when the robot is close to a collision state and raised to its standard value
          // when the robot is not close to a collision.
    None // Doesn't incorporate environment collision avoidance
}

#[derive(Clone,Debug,PartialEq)]
pub enum ObjectiveVariant {
    PositionMatch,
    OrientationMatch,
    PositionLiveliness,
    OrientationLiveliness,
    PositionMirroring,
    OrientationMirroring,
    PositionBounding,
    OrientationBounding,
    JointMatch,
    JointLiveliness,
    JointMirroring,
    JointLimits,
    NNSelfCollision,
    EnvCollision,
    MinimizeVelocity,
    MinimizeAcceleration,
    MinimizeJerk,
    RelativeMotionLiveliness,
    RootPositionLiveliness,
    Gravity,
    MacroSmoothness,
    None
}

impl From<ControlMode> for String {
    fn from(mode: ControlMode) -> String {
        match mode {
            ControlMode::Absolute => String::from("absolute"),
            ControlMode::Relative => String::from("relative")
        }
    }
}

impl From<String> for ControlMode {
    fn from(mode: String) -> ControlMode {
        match mode.as_str() {
            "absolute" => ControlMode::Absolute,
            "relative" => ControlMode::Relative,
            _ => ControlMode::Absolute // Default to Absolute
        }
    }
}

impl From<EnvironmentMode> for String {
    fn from(mode: EnvironmentMode) -> String {
        match mode {
            EnvironmentMode::ECA => String::from("ECA"),
            EnvironmentMode::ECA3 => String::from("ECA3"),
            EnvironmentMode::ECAA => String::from("ECAA"),
            EnvironmentMode::None => String::from("None")
        }
    }
}

impl From<String> for EnvironmentMode {
    fn from(mode: String) -> EnvironmentMode {
        match mode.as_str() {
            "ECA" => EnvironmentMode::ECA,
            "ECA3" => EnvironmentMode::ECA3,
            "ECAA" => EnvironmentMode::ECAA,
            "None" => EnvironmentMode::None,
            _ => EnvironmentMode::None // Default to None
        }
    }
}

impl From<ObjectiveVariant> for String {
    fn from(variant: ObjectiveVariant) -> String {
        match variant {
            ObjectiveVariant::PositionMatch => String::from("position_match"),
            ObjectiveVariant::OrientationMatch => String::from("orientation_match"),
            ObjectiveVariant::PositionLiveliness => String::from("position_liveliness"),
            ObjectiveVariant::OrientationLiveliness => String::from("orientation_liveliness"),
            ObjectiveVariant::PositionMirroring => String::from("position_mirroring"),
            ObjectiveVariant::OrientationMirroring => String::from("orientation_mirroring"),
            ObjectiveVariant::PositionBounding => String::from("position_bounding"),
            ObjectiveVariant::OrientationBounding => String::from("orientation_bounding"),
            ObjectiveVariant::JointMatch => String::from("joint_match"),
            ObjectiveVariant::JointLiveliness => String::from("joint_liveliness"),
            ObjectiveVariant::JointMirroring => String::from("joint_mirroring"),
            ObjectiveVariant::JointLimits => String::from("joint_limits"),
            ObjectiveVariant::NNSelfCollision => String::from("nn_collision"),
            ObjectiveVariant::EnvCollision => String::from("env_collision"),
            ObjectiveVariant::MinimizeVelocity => String::from("min_velocity"),
            ObjectiveVariant::MinimizeAcceleration => String::from("min_acceleration"),
            ObjectiveVariant::MinimizeJerk => String::from("min_jerk"),
            ObjectiveVariant::RelativeMotionLiveliness => String::from("relative_motion_liveliness"),
            ObjectiveVariant::RootPositionLiveliness => String::from("base_link_position_liveliness"),
            ObjectiveVariant::Gravity => String::from("gravity"),
            ObjectiveVariant::MacroSmoothness => String::from("macro_smoothness"),
            ObjectiveVariant::None => String::from("None")
        }
    }
}

impl From<String> for ObjectiveVariant {
    fn from(variant: String) -> ObjectiveVariant {
        match variant.as_str() {
            "position_match" => ObjectiveVariant::PositionMatch,
            "orientation_match" => ObjectiveVariant::OrientationMatch,
            "position_liveliness" => ObjectiveVariant::PositionLiveliness,
            "orientation_liveliness" => ObjectiveVariant::OrientationLiveliness,
            "position_bounding" => ObjectiveVariant::PositionBounding,
            "orientation_bounding" => ObjectiveVariant::OrientationBounding,
            "position_mirroring" => ObjectiveVariant::PositionMirroring,
            "orientation_mirroring" => ObjectiveVariant::OrientationMirroring,
            "joint_match" => ObjectiveVariant::JointMatch,
            "joint_liveliness" => ObjectiveVariant::JointLiveliness,
            "joint_mirroring" => ObjectiveVariant::JointMirroring,
            "joint_limits" => ObjectiveVariant::JointLimits,
            "nn_collision" => ObjectiveVariant::NNSelfCollision,
            "env_collision" => ObjectiveVariant::EnvCollision,
            "min_velocity" => ObjectiveVariant::MinimizeVelocity,
            "min_acceleration" => ObjectiveVariant::MinimizeAcceleration,
            "min_jerk" => ObjectiveVariant::MinimizeJerk,
            "relative_motion_liveliness" => ObjectiveVariant::RelativeMotionLiveliness,
            "base_link_position_liveliness" => ObjectiveVariant::RootPositionLiveliness,
            "gravity" => ObjectiveVariant::Gravity,
            "macro_smoothness" => ObjectiveVariant::MacroSmoothness,
            "None" => ObjectiveVariant::None,
            _ => ObjectiveVariant::None // Default to None
        }
    }
}
