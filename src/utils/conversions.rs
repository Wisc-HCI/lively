use nalgebra::{Vector3};
// use crate::utils::settings::{*};/

// pub fn control_mode_to_string(mode: ControlMode) -> String {
//     let mode_control: String;
//     match mode {
//         ControlMode::Relative => mode_control = String::from("relative"),
//         ControlMode::Absolute => mode_control = String::from("absolute")
//     }
//     return mode_control
// }
//
// pub fn environment_mode_to_string(mode: EnvironmentMode) -> String {
//     let mode_environment: String;
//     match mode {
//         EnvironmentMode::ECA => mode_environment = String::from("ECA"),
//         EnvironmentMode::ECA3 => mode_environment = String::from("ECA3"),
//         EnvironmentMode::ECAA => mode_environment = String::from("ECAA"),
//         EnvironmentMode::None => mode_environment = String::from("None")
//     }
//     return mode_environment
// }
//
// pub fn string_to_control_mode(string: String) -> ControlMode {
//     let mut mode_control: ControlMode;
//     match string {
//         String::from("absolute") => mode_control = ControlMode::Absolute,
//         String::from("belative") => mode_control = ControlMode::Relative,
//         _ => mode_control = ControlMode::Absolute // Default to Absolute
//     }
//     return mode_control;
// }
//
// pub fn string_to_environment_mode(string: String) -> EnvironmentMode {
//     let mut mode_environment: EnvironmentMode;
//     match string {
//         String::from("ECA") => mode_environment = EnvironmentMode::ECA,
//         String::from("ECA3") => mode_environment = EnvironmentMode::ECA3,
//         String::from("ECAA") => mode_environment = EnvironmentMode::ECAA,
//         String::from("None") => mode_environment = EnvironmentMode::None,
//         _ => mode_environment = EnvironmentMode::None // Default to None
//     }
//     return mode_environment;
// }

pub fn disp_offsets_to_vec(disp_offsets: Vec<Vector3<f64>>) -> Vec<Vec<f64>> {
    let mut disp_offsets_vec: Vec<Vec<f64>> = Vec::new();
    for i in 0..disp_offsets.len() {
        disp_offsets_vec.push(vec![disp_offsets[i][0],disp_offsets[i][1],disp_offsets[i][2]]);
    }
    return disp_offsets_vec;
}

pub fn displacements_to_vec(displacements: Vec<Vec<Vector3<f64>>>) -> Vec<Vec<Vec<f64>>> {
    let mut displacements_vec: Vec<Vec<Vec<f64>>> = Vec::new();
    for i in 0..displacements.len() {
        let mut displacement_series: Vec<Vec<f64>> = Vec::new();
        for j in 0..displacements[i].len() {
            displacement_series.push(vec![displacements[i][j][0],displacements[i][j][1],displacements[i][j][2]]);
        }
        displacements_vec.push(displacement_series);
    }
    return displacements_vec;
}

pub fn vec_to_disp_offsets(disp_offsets: Vec<Vec<f64>>) -> Vec<Vector3<f64>> {
    let mut disp_offset_vectors: Vec<Vector3<f64>> = Vec::new();
    for i in 0..disp_offsets.len() {
        disp_offset_vectors.push(Vector3::new( disp_offsets[i][0], disp_offsets[i][1], disp_offsets[i][2]));
    }
    return disp_offset_vectors;
}

pub fn vec_to_displacements(displacements: Vec<Vec<Vec<f64>>>) -> Vec<Vec<Vector3<f64>>> {
    let mut displacement_vectors: Vec<Vec<Vector3<f64>>> = Vec::new();
    for i in 0..displacements.len() {
        let vec3_vec: Vec<Vector3<f64>> = Vec::new();
        displacement_vectors.push(vec3_vec);
        for j in 0..displacements[i].len() {
            displacement_vectors[i].push( Vector3::new(displacements[i][j][0], displacements[i][j][1], displacements[i][j][2] ) );
        }
    }
    return displacement_vectors;
}
