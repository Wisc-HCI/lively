use crate::groove::vars::RelaxedIKVars;
use crate::groove::objective::{ObjectiveTrait, groove_loss};
use nalgebra::geometry::UnitQuaternion;
use nalgebra::Vector3;

pub struct Gravity {
    pub arm_idx: usize,
    pub joint_idx: usize,
}

impl Gravity {
    pub fn new(indices: Vec<usize>) -> Self {
        Self {
            arm_idx: indices[0],
            joint_idx: indices[1],
        }
    }
}

impl ObjectiveTrait for Gravity {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        if is_core == false {
            let gravity_goal = v.frames_core[self.arm_idx].0[self.joint_idx][2] - 0.4;
            x_val = (gravity_goal - frames[self.arm_idx].0[self.joint_idx][2]).abs();
        }
        // println!("PositionLiveliness loss: {:?}",groove_loss(x_val, 0., 2, 3.5, 0.00005, 4));
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}
