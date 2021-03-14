use crate::utils::goals::*;
use crate::utils::transformations::*;
// use std::cmp;
use crate::groove::vars::RelaxedIKVars;
use nalgebra::geometry::{Point3, Quaternion, UnitQuaternion};
use nalgebra::{one, Vector3};
use ncollide3d::{query, shape};
use std::ops::Deref;
// use time::PreciseTime;

pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -((-(x_val - t).powi(d)) / (2.0 * c.powi(2))).exp() + f * (x_val - t).powi(g)
}

pub fn groove_loss_derivative(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -((-(x_val - t).powi(d)) / (2.0 * c.powi(2))).exp()
        * ((-d as f64 * (x_val - t)) / (2.0 * c.powi(2)))
        + g as f64 * f * (x_val - t)
}

pub trait ObjectiveTrait {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64;
    fn gradient(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call(x, v, frames, is_core);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000000001;
            let frames_h = v.robot.get_frames(x_h.as_slice());
            let f_h = self.call(x_h.as_slice(), v, &frames_h, is_core);
            grad.push((-f_0 + f_h) / 0.000000001);
        }

        (f_0, grad)
    }
    fn gradient_type(&self) -> usize {
        return 1;
    } // manual diff = 0, finite diff = 1
}

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

pub struct NNSelfCollision;
impl ObjectiveTrait for NNSelfCollision {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut input_vec: Vec<f64> = Vec::new();
        let xvec = x.to_vec();
        for i in 3..xvec.len() {
            input_vec.push(xvec[i])
        }
        let x_val = v.collision_nn.predict(&input_vec);
        // println!("NNSelfCollision error: {:?}",x_val);
        groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }

    fn gradient(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> (f64, Vec<f64>) {
        let mut input_vec: Vec<f64> = Vec::new();
        let xvec = x.to_vec();
        for i in 3..xvec.len() {
            input_vec.push(xvec[i])
        }
        let (x_val, mut grad) = v.collision_nn.gradient(&input_vec);
        let g_prime = groove_loss_derivative(x_val, 0., 2, 2.1, 0.0002, 4);
        for i in 0..grad.len() {
            grad[i] *= g_prime;
        }
        (x_val, grad)
    }

    fn gradient_type(&self) -> usize {
        return 0;
    }
}

pub struct EnvCollision {
    pub arm_idx: usize,
}
impl EnvCollision {
    pub fn new(arm_idx: usize) -> Self {
        Self { arm_idx }
    }
}
impl ObjectiveTrait for EnvCollision {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        // let start = PreciseTime::now();\
        let mut x_val: f64 = 0.0;
        let link_radius = v.env_collision.link_radius;
        let penalty_cutoff: f64 = link_radius * 2.0;
        let a = penalty_cutoff.powi(2);
        for (option, _score) in &v.env_collision.active_obstacles[self.arm_idx] {
            if let Some(handle) = option {
                let mut sum: f64 = 0.0;
                let obstacle = v.env_collision.world.objects.get(*handle).unwrap();
                let last_elem = frames[self.arm_idx].0.len() - 1;
                for i in 0..last_elem {
                    let start_pt = Point3::from(frames[self.arm_idx].0[i]);
                    let end_pt = Point3::from(frames[self.arm_idx].0[i + 1]);
                    let segment = shape::Segment::new(start_pt, end_pt);
                    let segment_pos = one();
                    let dis = query::distance(
                        obstacle.position(),
                        obstacle.shape().deref(),
                        &segment_pos,
                        &segment,
                    ) - link_radius;
                    // println!("Obstacle: {}, Link: {}, Distance: {:?}", obstacle.data().name, i, dis);
                    sum += a / (dis + link_radius).powi(2);
                }
                // println!("OBJECTIVE -> {:?}, Sum: {:?}", obstacle.data().name, sum);
                x_val += sum;
            }
        }

        // let end = PreciseTime::now();
        // println!("Obstacles calculating takes {}", start.to(end));
        // println!("EnvCollision error: {:?}",x_val);
        groove_loss(x_val, 0., 2, 3.5, 0.00005, 4)
    }
}

pub struct JointLimits;
impl ObjectiveTrait for JointLimits {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.lower_bounds.len() {
            let l = v.robot.lower_bounds[i];
            let u = v.robot.upper_bounds[i];
            let range = u - l;
            match range {
                // In cases where the upper and lower limits are the same,
                // just compare the lower limit to the x value.
                range if range == 0.0 => sum += a * (x[i] - l).abs().powi(50),
                // Otherwise, compare as normal
                _ => {
                    let r = (x[i] - l) / (u - l);
                    let n = 2.0 * (r - 0.5);
                    sum += a * n.powi(50);
                }
            }
        }
        // println!("JointLimits error: {:?}",sum);
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
}

pub struct MinimizeVelocity;
impl ObjectiveTrait for MinimizeVelocity {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 3..x.len() {
            x_val += (x[i] - v.xopt[i - 3]).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("MinimizeVelocity error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeAcceleration;
impl ObjectiveTrait for MinimizeAcceleration {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 3..x.len() {
            let v1 = x[i] - v.xopt[i - 3];
            let v2: f64;
            if is_core {
                v2 = v.xopt[i - 3] - v.history_core.prev1[i - 3];
            } else {
                v2 = v.xopt[i - 3] - v.history.prev1[i - 3];
            }
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("MinimizeAcceleration error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeJerk;
impl ObjectiveTrait for MinimizeJerk {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 3..x.len() {
            let v1 = x[i] - v.xopt[i - 3];
            let v2: f64;
            let v3: f64;
            if is_core {
                v2 = v.xopt[i - 3] - v.history_core.prev1[i - 3];
                v3 = v.history_core.prev1[i - 3] - v.history_core.prev2[i - 3];
            } else {
                v2 = v.xopt[i - 3] - v.history.prev1[i - 3];
                v3 = v.history.prev1[i - 3] - v.history.prev2[i - 3];
            }
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("MinimizeJerk error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

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
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val: f64 = 0.0;
        if is_core {
            match v.goals[self.goal_idx].value {
                // goal must be a vector
                Goal::Scalar(offset) => {
                    // The error is the difference between the current value and the position along the source-target vector.
                    let src_pos = frames[self.arm_2_idx].0[self.joint_2_idx];
                    // pos_1 is the source position
                    let target_pos_orig = v.frames_core[self.arm_1_idx].0[self.joint_1_idx];
                    // target_pos_orig is the position for target from the core round
                    let unit_vector = (target_pos_orig - src_pos).normalize();
                    // diff is the difference between the above. Used for generating the "vector"
                    let target_pos = unit_vector * offset + target_pos_orig;
                    // target position is the scaled offset applied to the original target position
                    x_val = (src_pos - target_pos).norm();
                }
                _ => println!(
                    "Mismatched objective goals for objective with goal idx {:?}",
                    self.goal_idx
                ), // Some odd condition where incorrect input was provided
            }
        }
        // println!("RelativeMotionLiveliness error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
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

pub struct PositionBounding {
    // Bounds the position within a region according to the input provided in goals.
    pub goal_idx: usize,
    pub arm_idx: usize,
    pub joint_idx: usize,
}
impl PositionBounding {
    pub fn new(goal_idx: usize, indices: Vec<usize>) -> Self {
        Self {
            goal_idx,
            arm_idx: indices[0],
            joint_idx: indices[1],
        }
    }
}
impl ObjectiveTrait for PositionBounding {
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
                    x_val = (Vector3::new(x[0], x[1], x[2]) - noise_vec).norm();
                }
                // Ignore if it isn't
                _ => println!(
                    "Mismatched objective goals for objective with goal idx {:?}",
                    self.goal_idx
                ),
            }
        }
        // println!("RootPositionLiveliness error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}
