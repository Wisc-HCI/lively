use crate::utils::goals::*;
use crate::utils::transformations::*;
use crate::groove::vars::RelaxedIKVars;
use crate::groove::objective::{ObjectiveTrait, groove_loss};
use nalgebra::geometry::{Quaternion, UnitQuaternion};
use nalgebra::{Vector3};

pub struct PositionMatch {
    pub goal_idx: usize,
    pub arm_idx: usize,
    pub joint_idx: usize,
}
impl PositionMatch {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_idx: indices[0],
            joint_idx: indices[1],
        }
    }
}
impl ObjectiveTrait for PositionMatch {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        // Double-check that the goal is a 3-vector
        match v.goals[self.goal_idx].value {
            Goal::Vector(goal_vec) => {
                x_val = (frames[self.arm_idx].0[self.joint_idx] - goal_vec).norm();
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("PositionMatch error: {:?}",x_val);
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct OrientationMatch {
    pub goal_idx: usize,
    pub arm_idx: usize,
    pub joint_idx: usize,
}
impl OrientationMatch {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_idx: indices[0],
            joint_idx: indices[1],
        }
    }
}
impl ObjectiveTrait for OrientationMatch {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let tmp = Quaternion::new(
            -frames[self.arm_idx].1[self.joint_idx].w,
            -frames[self.arm_idx].1[self.joint_idx].i,
            -frames[self.arm_idx].1[self.joint_idx].j,
            -frames[self.arm_idx].1[self.joint_idx].k,
        );
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);
        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            Goal::Quaternion(goal_quat) => {
                let disp = angle_between(goal_quat, frames[self.arm_idx].1[self.joint_idx]);
                let disp2 = angle_between(goal_quat, ee_quat2);
                x_val = disp.min(disp2);
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("OrientationMatch error: {:?}",x_val);
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct JointMatch {
    // Sets a joint to a value given in scalar goal
    pub goal_idx: usize,
    pub joint_idx: usize,
}
impl JointMatch {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            joint_idx: indices[0],
        }
    }
}
impl ObjectiveTrait for JointMatch {
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
            Goal::Scalar(goal_val) => {
                // The error is the difference between the current value and the noise-augmented rotation solved previously w/o noise.
                // NOTE: xopt_core.len() == joints.len(), whereas x.len() == joints.len()+3
                x_val = (goal_val - x[self.joint_idx + 3]).abs();
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("JointMatch error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
    }
}

pub struct RootPositionMatch {
    // Adds position liveliness to the root node (first three entries in x are these values)
    pub goal_idx: usize,
}
impl RootPositionMatch {
    pub fn new(goal_idx: usize) -> Self {
        Self { goal_idx }
    }
}
impl ObjectiveTrait for RootPositionMatch {
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
            Goal::Vector(goal_vec) => {
                // The error is the difference between the current value and the goal value.
                x_val = (goal_vec - Vector3::new(x[0], x[1], x[2])).norm();
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("RootPositionLiveliness error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct DistanceMatch {
    // Adds position liveliness to the root node (first three entries in x are these values)
    pub goal_idx: usize,
    pub arm_1_idx: usize,
    pub arm_2_idx: usize,
    pub joint_1_idx: usize,
    pub joint_2_idx: usize,
}
impl DistanceMatch {
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
impl ObjectiveTrait for DistanceMatch {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            // goal must be a scalar
            Goal::Scalar(dist) => {
                // The error is the difference between the current value and the noise-augmented rotation solved previously w/o noise.
                // NOTE: xopt_core.len() == joints.len(), whereas x.len() == joints.len()+3
                let arm_1_pos = frames[self.arm_1_idx].0[self.joint_1_idx];
                let arm_2_pos = frames[self.arm_2_idx].0[self.joint_2_idx];
                x_val = ((arm_1_pos - arm_2_pos).norm() - dist).abs();
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }
        // println!("DistanceMatch error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}
