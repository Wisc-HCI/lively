use crate::utils::goals::*;
use crate::utils::transformations::*;
use crate::groove::vars::RelaxedIKVars;
use crate::groove::objective::{ObjectiveTrait, groove_loss};
use nalgebra::geometry::{Quaternion, UnitQuaternion};
use nalgebra::{Vector3};

pub struct PositionMirroring {
    // Matches the position between two joints, with a difference according to the Vector3 provided in goals.
    pub goal_idx: usize,
    pub arm_1_idx: usize,
    pub arm_2_idx: usize,
    pub joint_1_idx: usize,
    pub joint_2_idx: usize,
}
impl PositionMirroring {
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
impl ObjectiveTrait for PositionMirroring {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            // goal must be a vector
            Goal::Vector(offset_vec) => {
                // The error is the difference between the current value and the noise-augmented rotation solved previously w/o noise.
                // NOTE: xopt_core.len() == joints.len(), whereas x.len() == joints.len()+3
                let arm_1_pos = frames[self.arm_1_idx].0[self.joint_1_idx];
                let arm_2_pos = frames[self.arm_2_idx].0[self.joint_2_idx];
                x_val = ((arm_1_pos - arm_2_pos) - offset_vec).norm();
            }
            // If there is no goal, assume it is zero
            Goal::None => {
                let arm_1_pos = frames[self.arm_1_idx].0[self.joint_1_idx];
                let arm_2_pos = frames[self.arm_2_idx].0[self.joint_2_idx];
                x_val = (arm_1_pos - arm_2_pos).norm();
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("PositionMirroring error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct OrientationMirroring {
    // Matches the orientation between two joints, with a difference according to the Vector3 provided in goals.
    pub goal_idx: usize,
    pub arm_1_idx: usize,
    pub arm_2_idx: usize,
    pub joint_1_idx: usize,
    pub joint_2_idx: usize,
}
impl OrientationMirroring {
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
impl ObjectiveTrait for OrientationMirroring {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let tmp = Quaternion::new(
            -frames[self.arm_2_idx].1[self.joint_2_idx].w,
            -frames[self.arm_2_idx].1[self.joint_2_idx].i,
            -frames[self.arm_2_idx].1[self.joint_2_idx].j,
            -frames[self.arm_2_idx].1[self.joint_2_idx].k,
        );
        let ee_2_quat2 = UnitQuaternion::from_quaternion(tmp);
        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            // goal must be a vector
            Goal::Vector(offset_ori) => {
                // The error is the difference between the current value and the noise-augmented rotation solved previously w/o noise.
                // NOTE: xopt_core.len() == joints.len(), whereas x.len() == joints.len()+3
                let offset_arm_1 = quaternion_exp(
                    quaternion_log(v.frames_core[self.arm_1_idx].1[self.joint_1_idx]) + offset_ori,
                );
                let disp = angle_between(offset_arm_1, frames[self.arm_2_idx].1[self.joint_2_idx]);
                let disp2 = angle_between(offset_arm_1, ee_2_quat2);
                x_val = disp.min(disp2);
            }
            // If there is no goal, assume it is zero
            Goal::None => {
                let disp = angle_between(
                    v.frames_core[self.arm_1_idx].1[self.joint_1_idx],
                    frames[self.arm_2_idx].1[self.joint_2_idx],
                );
                let disp2 = angle_between(
                    v.frames_core[self.arm_1_idx].1[self.joint_1_idx],
                    ee_2_quat2,
                );
                x_val = disp.min(disp2);
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("OrientationMirroring error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct JointMirroring {
    // Match joint values according to the difference specified in goals
    pub goal_idx: usize,
    pub joint_1_idx: usize,
    pub joint_2_idx: usize,
}
impl JointMirroring {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            joint_1_idx: indices[0],
            joint_2_idx: indices[1],
        }
    }
}
impl ObjectiveTrait for JointMirroring {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            // goal must be a vector
            Goal::Scalar(offset) => {
                // The error is the difference between the current value and the noise-augmented rotation solved previously w/o noise.
                // NOTE: xopt_core.len() == joints.len(), whereas x.len() == joints.len()+3
                x_val = ((x[self.joint_1_idx + 3] - x[self.joint_2_idx + 3]) - offset).abs();
            }
            // If there is no goal, assume it is zero
            Goal::None => {
                x_val = (x[self.joint_1_idx + 3] - x[self.joint_2_idx + 3]).abs();
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("JointMirroring error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
    }
}
