use crate::groove::vars::RelaxedIKVars;
use nalgebra::geometry::{UnitQuaternion};
use nalgebra::{Vector3};

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

pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -((-(x_val - t).powi(d)) / (2.0 * c.powi(2))).exp() + f * (x_val - t).powi(g)
}

pub fn groove_loss_derivative(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -((-(x_val - t).powi(d)) / (2.0 * c.powi(2))).exp()
        * ((-d as f64 * (x_val - t)) / (2.0 * c.powi(2)))
        + g as f64 * f * (x_val - t)
}
