// use crate::groove::{vars};
// use crate::groove::env_collision::{*};
use crate::utils::transformations::{*};
use crate::utils::goals::{*};
// use std::cmp;
use crate::groove::vars::RelaxedIKVars;
use nalgebra::{Vector3, one};
use nalgebra::geometry::{Point3, UnitQuaternion, Quaternion};
use ncollide3d::{shape, query};
use std::ops::Deref;
// use time::PreciseTime;

pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x_val - t).powi(g)
}

pub fn groove_loss_derivative(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() *  ((-d as f64 * (x_val - t)) /  (2.0 * c.powi(2))) + g as f64 * f * (x_val - t)
}

pub trait ObjectiveTrait {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64;
    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64;
    fn gradient(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call(x, v, frames);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000000001;
            let frames_h = v.robot.get_frames_immutable(x_h.as_slice());
            let f_h = self.call(x_h.as_slice(), v, &frames_h);
            grad.push( (-f_0 + f_h) / 0.000000001);
        }

        (f_0, grad)
    }
    fn gradient_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call_lite(x, v, ee_poses);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.0000001;
            let ee_poses_h = v.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
            let f_h = self.call_lite(x_h.as_slice(), v, &ee_poses_h);
            grad.push( (-f_0 + f_h) / 0.0000001);
        }

        (f_0, grad)
    }
    fn gradient_type(&self) -> usize {return 1}  // manual diff = 0, finite diff = 1
}

pub struct MatchEEPosGoals {
    pub goal_idx: usize,
    pub arm_idx: usize
}
impl MatchEEPosGoals {
    pub fn new(goal_idx: usize, arm_idx: usize) -> Self {Self{goal_idx, arm_idx}}
}
impl ObjectiveTrait for MatchEEPosGoals {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let mut x_val: f64 = 0.0;
        // Double-check that the goal is a 3-vector
        match v.goals[self.goal_idx].value {
            Goal::Vector(goal_vec) => {
                x_val = ( frames[self.arm_idx].0[last_elem] - goal_vec ).norm();
            },
            _ => {} // Some odd condition where incorrect input was provided
        }

        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            Goal::Vector(goal_vec) => {
                 x_val = ( ee_poses[self.arm_idx].0 - goal_vec ).norm();
            },
            _ => {} // Some odd condition where incorrect input was provided
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct MatchEEQuatGoals {
    pub goal_idx: usize,
    pub arm_idx: usize
}
impl MatchEEQuatGoals {
    pub fn new(goal_idx: usize, arm_idx: usize) -> Self {Self{goal_idx, arm_idx}}
}
impl ObjectiveTrait for MatchEEQuatGoals {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let last_elem = frames[self.arm_idx].1.len() - 1;
        let tmp = Quaternion::new(-frames[self.arm_idx].1[last_elem].w, -frames[self.arm_idx].1[last_elem].i, -frames[self.arm_idx].1[last_elem].j, -frames[self.arm_idx].1[last_elem].k);
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);
        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            Goal::Quaternion(goal_quat) => {
                let disp = angle_between(goal_quat, frames[self.arm_idx].1[last_elem]);
                let disp2 = angle_between(goal_quat, ee_quat2);
                x_val = disp.min(disp2);
            },
            _ => {} // Some odd condition where incorrect input was provided
        }

        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let tmp = Quaternion::new(-ee_poses[self.arm_idx].1.w, -ee_poses[self.arm_idx].1.i, -ee_poses[self.arm_idx].1.j, -ee_poses[self.arm_idx].1.k);
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);

        let mut x_val: f64 = 0.0;
        match v.goals[self.goal_idx].value {
            Goal::Quaternion(goal_quat) => {
                let disp = angle_between(goal_quat, ee_poses[self.arm_idx].1);
                let disp2 = angle_between(goal_quat, ee_quat2);
                x_val = disp.min(disp2);
            },
            _ => {} // Some odd condition where incorrect input was provided
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct NNSelfCollision;
impl ObjectiveTrait for NNSelfCollision {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = v.collision_nn.predict(&x.to_vec());
        groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = v.collision_nn.predict(&x.to_vec());
        groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }

    fn gradient(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> (f64, Vec<f64>) {
        let (x_val, mut grad) = v.collision_nn.gradient(&x.to_vec());
        let g_prime = groove_loss_derivative(x_val, 0., 2, 2.1, 0.0002, 4);
        for i in 0..grad.len() {
            grad[i] *= g_prime;
        }
        (x_val, grad)
    }

    fn gradient_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> (f64, Vec<f64>) {
        let (x_val, mut grad) = v.collision_nn.gradient(&x.to_vec());
        let g_prime = groove_loss_derivative(x_val, 0., 2, 2.1, 0.0002, 4);
        for i in 0..grad.len() {
            grad[i] *= g_prime;
        }
        (x_val, grad)
    }

    fn gradient_type(&self) -> usize {return 0}
}

pub struct EnvCollision {
    pub arm_idx: usize
}
impl EnvCollision {
    pub fn new(arm_idx: usize) -> Self {Self{arm_idx}}
}
impl ObjectiveTrait for EnvCollision {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        // let start = PreciseTime::now();\
        let mut x_val: f64 = 0.0;
        let link_radius = v.env_collision.link_radius;
        let penalty_cutoff: f64 = link_radius * 2.0;
        let a = penalty_cutoff.powi(2);
        for (option, score) in &v.env_collision.active_obstacles[self.arm_idx] {
            if let Some(handle) = option {
                let mut sum: f64 = 0.0;
                let obstacle = v.env_collision.world.objects.get(*handle).unwrap();
                let last_elem = frames[self.arm_idx].0.len() - 1;
                for i in 0..last_elem {
                    let start_pt = Point3::from(frames[self.arm_idx].0[i]);
                    let end_pt = Point3::from(frames[self.arm_idx].0[i + 1]);
                    let segment = shape::Segment::new(start_pt, end_pt);
                    let segment_pos = one();
                    let dis = query::distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius;
                    // println!("Obstacle: {}, Link: {}, Distance: {:?}", obstacle.data().name, i, dis);
                    sum += a / (dis + link_radius).powi(2);
                }
                // println!("OBJECTIVE -> {:?}, Sum: {:?}", obstacle.data().name, sum);
                x_val += sum;
            }
        }

        // let end = PreciseTime::now();
        // println!("Obstacles calculating takes {}", start.to(end));

        groove_loss(x_val, 0., 2, 3.5, 0.00005, 4)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let x_val = 1.0; // placeholder
        groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }
}

pub struct JointLimits;
impl ObjectiveTrait for JointLimits {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.num_dof {
            let l = v.robot.bounds[i][0];
            let u = v.robot.bounds[i][1];
            let r = (x[i] - l) / (u - l);
            let n = 2.0 * (r - 0.5);
            sum += a*n.powf(50.);
        }
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.85;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.num_dof {
            let l = v.robot.bounds[i][0];
            let u = v.robot.bounds[i][1];
            let r = (x[i] - l) / (u - l);
            let n = 2.0 * (r - 0.5);
            sum += a*n.powi(50);
        }
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
}

pub struct MinimizeVelocity;
impl ObjectiveTrait for MinimizeVelocity {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
           x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
           x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

}

pub struct MinimizeAcceleration;
impl ObjectiveTrait for MinimizeAcceleration {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeJerk;
impl ObjectiveTrait for MinimizeJerk {
    fn call(&self, x: &[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &RelaxedIKVars, ee_poses: &Vec<(Vector3<f64>, UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}
