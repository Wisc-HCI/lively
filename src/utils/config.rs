use pyo3::prelude::*;
use nalgebra::{DMatrix, Vector3};
// use nalgebra::{DMatrix};
use crate::utils::shapes::{Sphere, Cuboid, PC};
use crate::utils::goals::GoalSpec;
use crate::utils::conversions::{*};
use crate::utils::settings::{*};

#[pyclass]
#[derive(Clone,Debug)]
pub struct ObjectiveSpec {
    pub variant: ObjectiveVariant,
    #[pyo3(get, set)]
    pub tag: String,
    #[pyo3(get, set)]
    pub indices: Vec<usize>,
    #[pyo3(get, set)]
    // Scale is used for single-dimension objectives, such as joint values
    pub scale: Option<f64>,
    #[pyo3(get, set)]
    // Shape is used for multi-dimension objectives, such as ee positions
    pub shape: Option<Vec<f64>>,
    #[pyo3(get, set)]
    pub frequency: Option<f64>
}

#[pymethods]
impl ObjectiveSpec {
    #[new]
    fn new(variant: String, tag: String, indices: Vec<usize>, scale: Option<f64>, shape: Option<Vec<f64>>, frequency: Option<f64>) -> Self {
        let variant_enum = ObjectiveVariant::from(variant);
        Self { variant: variant_enum, tag, indices, scale, shape, frequency }
    }
    #[getter]
    fn get_variant(&self) -> PyResult<String> {
        return Ok(String::from(self.variant.clone()))
    }
    #[setter]
    fn set_variant(&mut self, variant: String) -> PyResult<()> {
        self.variant = ObjectiveVariant::from(variant);
        return Ok(())
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct NNSpec {
    #[pyo3(get, set)]
    pub coefs: Vec<Vec<Vec<f64>>>,
    #[pyo3(get, set)]
    pub intercepts: Vec<Vec<f64>>,
    #[pyo3(get, set)]
    pub split_point: f64,
    // Non-Python, derived values
    pub coef_matrices: Vec<DMatrix<f64>>,
    pub intercept_vectors: Vec<DMatrix<f64>>
}

#[pymethods]
impl NNSpec {
    #[new]
    fn new(coefs: Vec<Vec<Vec<f64>>>, intercepts: Vec<Vec<f64>>, split_point: f64) -> Self {
        let mut coef_matrices: Vec<DMatrix<f64>> = Vec::new();
        for i in 0..coefs.len() {
            let mut m = DMatrix::from_element( coefs[i].len(), coefs[i][0].len(), 0.0 );
            for j in 0..coefs[i].len() {
                for k in 0..coefs[i][j].len() {
                    m[(j,k)] = coefs[i][j][k];
                }
            }
            coef_matrices.push(m);
        }
        let mut intercept_vectors: Vec<DMatrix<f64>> = Vec::new();
        for i in 0..intercepts.len() {
            let mut v = DMatrix::from_element(1, intercepts[i].len(),0.0);
            for j in 0..intercepts[i].len() {
                v[j] = intercepts[i][j];
            }
            intercept_vectors.push(v);
        }
        Self { coefs, intercepts, split_point, coef_matrices, intercept_vectors }
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct EnvironmentSpec {
    #[pyo3(get, set)]
    pub cuboids: Vec<Cuboid>,
    #[pyo3(get, set)]
    pub spheres: Vec<Sphere>,
    #[pyo3(get, set)]
    pub pcs: Vec<PC>
}

#[pymethods]
impl EnvironmentSpec {
    #[new]
    fn new(cuboids: Vec<Cuboid>, spheres: Vec<Sphere>, pcs: Vec<PC>) -> Self {
        Self { cuboids, spheres, pcs }
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct ModeConfig {
    #[pyo3(get, set)]
    pub name: String,
    #[pyo3(get, set)]
    pub goals: Vec<GoalSpec>
}

#[pymethods]
impl ModeConfig {
    #[new]
    fn new(name: String, goals: Vec<GoalSpec>) -> Self {
        Self { name, goals }
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct Config {
    #[pyo3(get, set)]
    pub axis_types: Vec<Vec<String>>,
    #[pyo3(get, set)]
    pub base_link_motion_bounds: Vec<[f64; 2]>,
    #[pyo3(get, set)]
    pub static_environment: EnvironmentSpec,
    #[pyo3(get, set)]
    pub ee_fixed_joints: Vec<String>,
    #[pyo3(get, set)]
    pub fixed_frame: String,
    #[pyo3(get, set)]
    pub joint_limits: Vec<[f64; 2]>,
    #[pyo3(get, set)]
    pub joint_names: Vec<Vec<String>>,
    #[pyo3(get, set)]
    pub joint_ordering: Vec<String>,
    #[pyo3(get, set)]
    pub joint_types: Vec<Vec<String>>,
    #[pyo3(get, set)]
    pub modes: Vec<ModeConfig>,
    #[pyo3(get, set)]
    pub nn_jointpoint: NNSpec,
    #[pyo3(get, set)]
    pub nn_main: NNSpec,
    #[pyo3(get, set)]
    pub objectives: Vec<ObjectiveSpec>,
    #[pyo3(get, set)]
    pub states: Vec<Vec<f64>>,
    #[pyo3(get, set)]
    pub robot_link_radius: f64,
    #[pyo3(get, set)]
    pub rot_offsets: Vec<Vec<Vec<f64>>>,
    #[pyo3(get, set)]
    pub starting_config: Vec<f64>,
    #[pyo3(get, set)]
    pub urdf: String,
    #[pyo3(get, set)]
    pub velocity_limits: Vec<f64>,
    // Non-py03 attributes
    pub disp_offsets: Vec<Vector3<f64>>,
    pub displacements: Vec<Vec<Vector3<f64>>>,
    pub mode_control: ControlMode,
    pub mode_environment: EnvironmentMode,
}

#[pymethods]
impl Config {
    #[new]
    fn new(axis_types: Vec<Vec<String>>, base_link_motion_bounds: Vec<[f64; 2]>,
           static_environment: EnvironmentSpec, ee_fixed_joints: Vec<String>, fixed_frame: String,
           joint_limits: Vec<[f64; 2]>, joint_names: Vec<Vec<String>>, joint_ordering: Vec<String>,
           joint_types: Vec<Vec<String>>, modes: Vec<ModeConfig>, mode_control: String, mode_environment: String,
           nn_jointpoint: NNSpec, nn_main: NNSpec, objectives: Vec<ObjectiveSpec>, states: Vec<Vec<f64>>,
           robot_link_radius: f64, rot_offsets: Vec<Vec<Vec<f64>>>, starting_config: Vec<f64>, urdf: String,
           velocity_limits: Vec<f64>, disp_offsets: Vec<Vec<f64>>, displacements: Vec<Vec<Vec<f64>>>) -> Self {

       let displacement_vectors: Vec<Vec<Vector3<f64>>> = vec_to_displacements(displacements);
       let disp_offset_vectors: Vec<Vector3<f64>> = vec_to_disp_offsets(disp_offsets);
       let mode_control_setting = ControlMode::from(mode_control);
       let mode_env_setting = EnvironmentMode::from(mode_environment);

       Self { axis_types, base_link_motion_bounds, static_environment, ee_fixed_joints,
              fixed_frame, joint_limits, joint_names, joint_ordering, joint_types, modes,
              mode_control: mode_control_setting, mode_environment: mode_env_setting,
              nn_jointpoint, nn_main, objectives, states, robot_link_radius, rot_offsets,
              starting_config, urdf, velocity_limits, disp_offsets: disp_offset_vectors,
              displacements: displacement_vectors }
    }

    #[getter]
    fn get_disp_offsets(&self) -> PyResult<Vec<Vec<f64>>> {
        return Ok(disp_offsets_to_vec(self.disp_offsets.clone()));
    }

    #[setter]
    fn set_disp_offsets(&mut self, disp_offsets: Vec<Vec<f64>>) -> PyResult<()> {
        self.disp_offsets = vec_to_disp_offsets(disp_offsets);
        Ok(())
    }

    #[getter]
    fn get_displacements(&self) -> PyResult<Vec<Vec<Vec<f64>>>> {
        return Ok(displacements_to_vec(self.displacements.clone()));
    }

    #[setter]
    fn set_displacements(&mut self, displacements: Vec<Vec<Vec<f64>>>) -> PyResult<()> {
        self.displacements = vec_to_displacements(displacements);
        Ok(())
    }

    #[getter]
    fn get_mode_control(&self) -> PyResult<String> {
        return Ok(String::from(self.mode_control.clone())); //Ok(control_mode_to_string(self.mode_control))
    }

    #[getter]
    fn get_mode_environment(&self) -> PyResult<String> {
        return Ok(String::from(self.mode_environment.clone())) // Ok(environment_mode_to_string(self.mode_environment))
    }

    #[setter]
    fn set_mode_control(&mut self, value:String) -> PyResult<()> {
        self.mode_control = ControlMode::from(value);
        return Ok(())
    }

    #[setter]
    fn set_mode_environment(&mut self, value:String) -> PyResult<()> {
        self.mode_environment = EnvironmentMode::from(value);
        return Ok(())
    }

    #[getter]
    fn get_default_goals(&self) -> PyResult<Vec<GoalSpec>> {
        return Ok(self.default_goals())
    }
}

impl Config {
    pub fn default_goals(&self) -> Vec<GoalSpec> {
        let mut default_goals:Vec<GoalSpec> = Vec::new();
        for mode_config in self.modes.clone() {
            match mode_config.name.as_str() {
                "default" => {
                    // Transfer contents to default goals
                    for goal in mode_config.goals.clone() {
                        default_goals.push(goal)
                    }
                    break;
                },
                _ => {}
            }
        }
        return default_goals;
    }
}
