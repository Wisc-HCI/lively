use crate::utils::goals::*;
use crate::utils::transformations::*;
use crate::groove::vars::RelaxedIKVars;
use crate::groove::objective::{ObjectiveTrait, groove_loss};
use nalgebra::geometry::{Quaternion, UnitQuaternion};
use nalgebra::{Vector3};

pub struct PositionLiveliness {
    // Adds position liveliness to the specified end effector
    pub goal_idx: usize,
    pub arm_idx: usize,
    pub joint_idx: usize,
}
impl PositionLiveliness {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_idx: indices[0],
            joint_idx: indices[1],
        }
    }
}
impl ObjectiveTrait for PositionLiveliness {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        if is_core == false {
            match v.liveliness.goals[self.goal_idx] {
                // The goal must be a 3-vector.
                Goal::Vector(noise_vec) => {
                    // The error is the difference between the current value and the noise-augmented position solved previously w/o noise.
                    x_val = (frames[self.arm_idx].0[self.joint_idx]
                        - (v.frames_core[self.arm_idx].0[self.joint_idx] + noise_vec))
                        .norm();
                    // println!("PositionLiveliness Objective enabled")
                }
                // Ignore if it isn't
                _ => println!(
                    "Mismatched objective goals for objective with goal idx {:?}",
                    self.goal_idx
                ),
            }
        }
        // println!("PositionLiveliness loss: {:?}",groove_loss(x_val, 0., 2, 3.5, 0.00005, 4));
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct OrientationLiveliness {
    // Adds orientation liveliness to the specified end effector
    pub goal_idx: usize,
    pub arm_idx: usize,
    pub joint_idx: usize,
}
impl OrientationLiveliness {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_idx: indices[0],
            joint_idx: indices[1],
        }
    }
}
impl ObjectiveTrait for OrientationLiveliness {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        // Since there are 2 ways to measure the distance around the unit sphere of orientations,
        // calculate actual orientation of the end effector both ways.
        let tmp = Quaternion::new(
            -frames[self.arm_idx].1[self.joint_idx].w,
            -frames[self.arm_idx].1[self.joint_idx].i,
            -frames[self.arm_idx].1[self.joint_idx].j,
            -frames[self.arm_idx].1[self.joint_idx].k,
        );
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);
        let mut x_val: f64 = 0.0;
        if is_core == false {
            match v.liveliness.goals[self.goal_idx] {
                // goal must be a vector
                Goal::Vector(noise_vec) => {
                    // The error is the difference between the current value and the noise-augmented rotation solved previously w/o noise.
                    // (v.frames_core[self.arm_idx].1[self.joint_idx] is the orientation from the previous "core solving" round)
                    let lively_goal = quaternion_exp(
                        quaternion_log(v.frames_core[self.arm_idx].1[self.joint_idx]) + noise_vec,
                    );
                    let disp = angle_between(lively_goal, frames[self.arm_idx].1[self.joint_idx]);
                    let disp2 = angle_between(lively_goal, ee_quat2);
                    x_val = disp.min(disp2);
                }
                _ => println!(
                    "Mismatched objective goals for objective with goal idx {:?}",
                    self.goal_idx
                ), // Some odd condition where incorrect input was provided
            }
        }
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct JointLiveliness {
    // Adds joint liveliness
    pub goal_idx: usize,
    pub joint_idx: usize,
}
impl JointLiveliness {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            joint_idx: indices[0],
        }
    }
}
impl ObjectiveTrait for JointLiveliness {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        if is_core == false {
            match v.liveliness.goals[self.goal_idx] {
                // goal must be a vector
                Goal::Scalar(noise_val) => {
                    // The error is the difference between the current value and the noise-augmented rotation solved previously w/o noise.
                    // NOTE: xopt_core.len() == joints.len(), whereas x.len() == joints.len()+3
                    let lively_goal = v.xopt_core[self.joint_idx] + noise_val;
                    x_val = (lively_goal - x[self.joint_idx + 3]).abs();
                }
                _ => println!(
                    "Mismatched objective goals for objective with goal idx {:?}",
                    self.goal_idx
                ), // Some odd condition where incorrect input was provided
            }
        }
        // println!("JointLiveliness error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
    }
}


pub struct RelativeMotionLiveliness {
    // Defining a vector line between two joints, this objective promotes lively motion of the second joint along that vector
    pub goal_idx: usize,
    pub arm_1_idx: usize,
    pub arm_2_idx: usize,
    pub joint_1_idx: usize,
    pub joint_2_idx: usize,
}
impl RelativeMotionLiveliness {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_1_idx: indices[0],
            joint_1_idx: indices[1],
            arm_2_idx: indices[2],
            joint_2_idx: indices[3],
        }
    }
}
impl ObjectiveTrait for RelativeMotionLiveliness {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        if is_core == false {
            match v.liveliness.goals[self.goal_idx] {
                // goal must be a vector
                Goal::Scalar(offset) => {
                    // The error is the difference between the current value and the position along the source-target vector.
                    let src_pos = v.frames_core[self.arm_2_idx].0[self.joint_2_idx];
                    // pos_1 is the source position
                    let target_pos_orig = v.frames_core[self.arm_1_idx].0[self.joint_1_idx];
                    // target_pos_orig is the position for target from the core round
                    let unit_vector = (target_pos_orig - src_pos).normalize();
                    // diff is the difference between the above. Used for generating the "vector"
                    let target_pos = unit_vector * offset + target_pos_orig;
                    // target position is the scaled offset applied to the original target position
                    x_val = (target_pos_orig - target_pos).norm();
                    // println!("{:?}, {:?} {:?}", target_pos_orig, target_pos, offset)
                }
                _ => println!(
                    "Mismatched objective goals for objective with goal idx {:?}",
                    self.goal_idx
                ), // Some odd condition where incorrect input was provided
            }
        }
        // println!("RelativeMotionLiveliness error: {:?}",x_val);
        // groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct RootPositionLiveliness {
    // Adds position liveliness to the root node (first three entries in x are these values)
    pub goal_idx: usize,
}
impl RootPositionLiveliness {
    pub fn new(goal_idx: usize) -> Self {
        Self { goal_idx }
    }
}
impl ObjectiveTrait for RootPositionLiveliness {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        if is_core == false {
            match v.liveliness.goals[self.goal_idx] {
                // The goal must be a 3-vector.
                Goal::Vector(noise_vec) => {
                    // The error is the difference between the current value and the noise-augmented position solved previously w/o noise.
                    x_val = (noise_vec - Vector3::new(x[0], x[1], x[2])).norm();
                }
                // Ignore if it isn't
                _ => println!(
                    "Mismatched objective goals for objective with goal idx {:?}",
                    self.goal_idx
                ),
            }
        }
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}
