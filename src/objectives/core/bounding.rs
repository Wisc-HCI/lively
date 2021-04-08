use crate::utils::goals::*;
use crate::groove::vars::RelaxedIKVars;
use crate::groove::objective::{ObjectiveTrait, groove_loss};
use nalgebra::geometry::{Isometry3, Point3, Translation3, UnitQuaternion};
use nalgebra::{Vector3};

pub struct PositionBounding {
    // Bounds the position within a region according to the input provided in goals.
    pub goal_idx: usize,
    pub arm_idx: usize,
    pub joint_idx: usize,
    pub shape: Vector3<f64>,
}
impl PositionBounding {
    pub fn new(goal_idx: usize, shape: Vector3<f64>, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_idx: indices[0],
            joint_idx: indices[1],
            shape,
        }
    }
}
impl ObjectiveTrait for PositionBounding {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        let position = Point3::from(frames[self.arm_idx].0[self.joint_idx]);

        match v.goals[self.goal_idx].value {
            // Goal must be a position/orientation pair
            Goal::Pose(pose_pair) => {
                // Translate position into coordinate frame of ellipse based on pose_pair
                let transform = Isometry3::from_parts(Translation3::from(pose_pair.0), pose_pair.1);
                let pos = transform.inverse_transform_point(&position);
                x_val = (pos[0].powi(1) / self.shape[0].powi(2)
                    + pos[1].powi(2) / self.shape[1].powi(2)
                    + pos[2].powi(2) / self.shape[2].powi(2))
                .powi(2)
            }
            // If there is no goal, assume it is the position/orientation from core
            Goal::None => {
                let pose_pair = (
                    v.frames_core[self.arm_idx].0[self.joint_idx],
                    v.frames_core[self.arm_idx].1[self.joint_idx],
                );
                // Translate position into coordinate frame of ellipse based on pose_pair
                let transform = Isometry3::from_parts(Translation3::from(pose_pair.0), pose_pair.1);
                let pos = transform.inverse_transform_point(&position);

                x_val = (pos[0].powi(1) / self.shape[0].powi(2)
                    + pos[1].powi(2) / self.shape[1].powi(2)
                    + pos[2].powi(2) / self.shape[2].powi(2))
                .powi(2)
            }
            _ => println!(
                "Mismatched objective goals for objective with goal idx {:?}",
                self.goal_idx
            ), // Some odd condition where incorrect input was provided
        }

        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct OrientationBounding {
    // Bounds the orientation within a region on the unit sphere according to the input provided in goals.
    pub goal_idx: usize,
    pub arm_idx: usize,
    pub joint_idx: usize,
}
impl OrientationBounding {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_idx: indices[0],
            joint_idx: indices[1],
        }
    }
}
impl ObjectiveTrait for OrientationBounding {
    fn call(
        &self,
        _x: &[f64],
        _v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let x_val: f64 = 0.0;
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}
