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
    EEPositionMatch,
    EEOrientationMatch,
    EEPositionLiveliness,
    EEOrientationLiveliness,
    EEPositionMirroring,
    EEOrientationMirroring,
    EEPositionBounding,
    EEOrientationBounding,
    JointMatch,
    JointLiveliness,
    JointMirroring,
    JointLimits,
    NNSelfCollision,
    EnvCollision,
    MinimizeVelocity,
    MinimizeAcceleration,
    MinimizeJerk,
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
            ObjectiveVariant::EEPositionMatch => String::from("ee_position_match"),
            ObjectiveVariant::EEOrientationMatch => String::from("ee_orientation_match"),
            ObjectiveVariant::EEPositionLiveliness => String::from("ee_position_liveliness"),
            ObjectiveVariant::EEOrientationLiveliness => String::from("ee_orientation_liveliness"),
            ObjectiveVariant::EEPositionMirroring => String::from("ee_position_mirroring"),
            ObjectiveVariant::EEOrientationMirroring => String::from("ee_orientation_mirroring"),
            ObjectiveVariant::EEPositionBounding => String::from("ee_position_bounding"),
            ObjectiveVariant::EEOrientationBounding => String::from("ee_orientation_bouding"),
            ObjectiveVariant::JointMatch => String::from("joint_match"),
            ObjectiveVariant::JointLiveliness => String::from("joint_liveliness"),
            ObjectiveVariant::JointMirroring => String::from("joint_mirroring"),
            ObjectiveVariant::JointLimits => String::from("joint_limits"),
            ObjectiveVariant::NNSelfCollision => String::from("nn_collision"),
            ObjectiveVariant::EnvCollision => String::from("env_collision"),
            ObjectiveVariant::MinimizeVelocity => String::from("min_velocity"),
            ObjectiveVariant::MinimizeAcceleration => String::from("min_acceleration"),
            ObjectiveVariant::MinimizeJerk => String::from("min_jerk"),
            ObjectiveVariant::None => String::from("None")
        }
    }
}

impl From<String> for ObjectiveVariant {
    fn from(variant: String) -> ObjectiveVariant {
        match variant.as_str() {
            "ee_position_match" => ObjectiveVariant::EEPositionMatch,
            "ee_orientation_match" => ObjectiveVariant::EEOrientationMatch,
            "ee_position_liveliness" => ObjectiveVariant::EEPositionLiveliness,
            "ee_orientation_liveliness" => ObjectiveVariant::EEOrientationLiveliness,
            "ee_position_bounding" => ObjectiveVariant::EEPositionBounding,
            "ee_orientation_bouding" => ObjectiveVariant::EEOrientationBounding,
            "joint_match" => ObjectiveVariant::JointMatch,
            "joint_liveliness" => ObjectiveVariant::JointLiveliness,
            "joint_mirroring" => ObjectiveVariant::JointMirroring,
            "joint_limits" => ObjectiveVariant::JointLimits,
            "nn_collision" => ObjectiveVariant::NNSelfCollision,
            "env_collision" => ObjectiveVariant::EnvCollision,
            "min_velocity" => ObjectiveVariant::MinimizeVelocity,
            "min_acceleration" => ObjectiveVariant::MinimizeAcceleration,
            "min_jerk" => ObjectiveVariant::MinimizeJerk,
            "None" => ObjectiveVariant::None,
            _ => ObjectiveVariant::None // Default to None
        }
    }
}
