use nalgebra::{Vector3};

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
