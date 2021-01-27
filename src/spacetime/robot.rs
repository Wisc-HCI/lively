use crate::spacetime::arm;
// use crate::utils::{geometry_utils, yaml_utils};
use crate::utils::config::Config;
use nalgebra::Vector3;
use pyo3::prelude::*;

#[pyclass]
#[derive(Clone, Debug)]
pub struct Robot {
    pub arms: Vec<arm::Arm>,
    #[pyo3(get, set)]
    pub joint_names: Vec<Vec<String>>,
    #[pyo3(get, set)]
    pub joint_ordering: Vec<String>,
    #[pyo3(get, set)]
    pub num_chains: usize,
    #[pyo3(get, set)]
    pub num_dof: usize,
    #[pyo3(get, set)]
    pub subchain_indices: Vec<Vec<usize>>,
    #[pyo3(get, set)]
    pub bounds: Vec< [f64; 2] >,
    #[pyo3(get, set)]
    pub lower_bounds: Vec<f64>,
    #[pyo3(get, set)]
    pub upper_bounds: Vec<f64>,
    // pub opt_lower_bounds: Vec<f64>,
    // pub opt_upper_bounds: Vec<f64>,
    #[pyo3(get, set)]
    pub velocity_limits: Vec<f64>
}

#[pymethods]
impl Robot {
    #[new]
    pub fn new(config: Config) -> Robot {
        let num_chains = config.axis_types.len();
        let num_dof = config.velocity_limits.len();

        // Get the base link motion limits:
        let x_lim: [f64; 2] = config.base_link_motion_bounds[0];
        let y_lim: [f64; 2] = config.base_link_motion_bounds[1];
        let z_lim: [f64; 2] = config.base_link_motion_bounds[2];

        let mut arms: Vec<arm::Arm> = Vec::new();
        for i in 0..num_chains {
            let a = arm::Arm::new(config.axis_types[i].clone(), config.displacements[i].clone(),
                                  config.disp_offsets[i].clone(), config.rot_offsets[i].clone(), config.joint_types[i].clone());
            arms.push(a);
        }

        let subchain_indices = Robot::get_subchain_indices(&config.joint_names, &config.joint_ordering);

        let mut upper_bounds: Vec<f64> = vec![x_lim[1],y_lim[1],z_lim[1]];
        let mut lower_bounds: Vec<f64> = vec![x_lim[0],y_lim[0],z_lim[0]];

        for i in 0..config.joint_limits.len() {
            upper_bounds.push(config.joint_limits[i][1].clone());
            lower_bounds.push(config.joint_limits[i][0].clone());
        }

        Robot{arms, joint_names: config.joint_names.clone(), joint_ordering: config.joint_ordering.clone(),
            num_chains, num_dof, subchain_indices, bounds: config.joint_limits.clone(), lower_bounds, upper_bounds,
            velocity_limits: config.velocity_limits.clone()}
    }

    pub fn x_as_subchains(&self, x: Vec<f64>) -> PyResult<Vec<Vec<f64>>> {
        Ok(self.split_into_subchains(x.as_slice()))
    }

}

impl Robot {

    pub fn split_into_subchains(&self, x: &[f64]) -> Vec<Vec<f64>>{
        let mut out_subchains: Vec<Vec<f64>> = Vec::new();
        for i in 0..self.num_chains {
            let mut chain: Vec<f64> = Vec::new();
            for index in self.subchain_indices[i].clone() {
                chain.push( x[index] );
            }
            out_subchains.push(chain);
        }
        out_subchains
    }

    pub fn get_frames(&self, x: &[f64]) -> Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> {
        let mut out: Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> = Vec::new();
        let subchains = self.split_into_subchains(x);
        let offset: Vector3<f64> = Vector3::new(x[0],x[1],x[2]);
        for i in 0..self.num_chains {
            let (positions,quaternions) = self.arms[i].get_frames( subchains[i].as_slice() );
            let mut offset_positions:Vec<nalgebra::Vector3<f64>> = Vec::new();
            for position in positions {
                offset_positions.push(position+offset);
            }
            out.push((offset_positions,quaternions));
        }
        out
    }

    pub fn get_ee_positions(&self, x: &[f64]) -> Vec<nalgebra::Vector3<f64>> {
        let mut out: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let subchains = self.split_into_subchains(x);
        let offset: Vector3<f64> = Vector3::new(x[0],x[1],x[2]);
        for i in 0..self.num_chains {
            out.push(self.arms[i].get_ee_position(subchains[i].as_slice())+offset);
        }
        out
    }

    pub fn get_ee_rot_mats(&self, x: &[f64]) -> Vec<nalgebra::Matrix3<f64>> {
        let mut out: Vec<nalgebra::Matrix3<f64>> = Vec::new();
        let subchains = self.split_into_subchains(x);
        for i in 0..self.num_chains {
            out.push(self.arms[i].get_ee_rot_mat(subchains[i].as_slice()));
        }
        out
    }

    pub fn get_ee_quats(&self, x: &[f64]) -> Vec<nalgebra::UnitQuaternion<f64>> {
        let mut out: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
        let subchains = self.split_into_subchains(x);
        for i in 0..self.num_chains {
            out.push(self.arms[i].get_ee_quat(subchains[i].as_slice()));
        }
        out
    }

    fn get_subchain_indices(joint_names: &Vec<Vec<String>>, joint_ordering: &Vec<String>) -> Vec<Vec<usize>> {
        let mut out: Vec<Vec<usize>> = Vec::new();

        for i in 0..joint_names.len() {
            let mut chain: Vec<usize> = Vec::new();
            for j in 0..joint_names[i].len() {
                let idx = Robot::get_index_from_joint_order(joint_ordering, &joint_names[i][j]);
                if  idx == 101010101010 {
                } else {
                    chain.push(idx);
                }
            }
            out.push(chain);
        }
        out
    }

    pub fn get_index_from_joint_order(joint_ordering: &Vec<String>, joint_name: &String) -> usize {
        for i in 0..joint_ordering.len() {
            if *joint_name == joint_ordering[i] {
                return i + 3
            }
        }
        101010101010
    }

}
