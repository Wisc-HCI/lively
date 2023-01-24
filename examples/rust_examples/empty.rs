use lively::lively::Solver;
use lively::objectives::core::base::CollisionAvoidanceObjective;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::core::matching::PositionMatchObjective;
use lively::objectives::objective::Objective;
use lively::utils::goals::Goal::ScalarRange;
use lively::utils::goals::{Goal};
use lively::utils::shapes::*;
use lively::utils::info::{*};
use lively::utils::shapes::Shape;
use nalgebra::base::Vector4;
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use nalgebra::geometry::Quaternion;
use nalgebra::geometry::Translation3;
use nalgebra::geometry::UnitQuaternion;
use std::collections::HashMap;
use rand::Rng;

use std::fs;

use std::time::{Instant};

// mod imgui_support;
// use imgui_support::ImguiManager;

// mod renderer;
// use puffin::set_scopes_on;
// use renderer::Renderer;


fn main() {
    let data =
        fs::read_to_string("./tests/basic.xml").expect("Something went wrong reading the file");

    let pos_match_obj =
        PositionMatchObjective::new("EE Position".to_string(), 20.0, "wrist_3_link".to_string());
    let col_avoid_obj = CollisionAvoidanceObjective::new("Collision Avoidance".to_string(), 10.0);
    let smooth_macro_obj = SmoothnessMacroObjective::new("Smoothness".to_string(), 10.0);
    let scalar_range_1 = (0.0, 0.0);
    let scalar_range_2 = (-0.15, 0.0);
    let scalar_range_3 = (0.0, 0.0);
    let scalar_range_4 = (0.0, 0.0);
    let scalar_range_5 = (0.0, 0.0);
    let scalar_range_6 = (0.0, 0.0);
    let scalar_range_vec: Vec<(f64, f64)> = vec![
        scalar_range_1,
        scalar_range_2,
        scalar_range_3,
        scalar_range_4,
        scalar_range_5,
        scalar_range_6,
    ];

    let mut objectives: HashMap<String,Objective> = HashMap::new();
    objectives.insert("iowsdsfhwe".into(),Objective::PositionMatch(pos_match_obj));
    objectives.insert("sdfsddsfes".into(),Objective::CollisionAvoidance(col_avoid_obj));
    objectives.insert("dfawdaseas".into(),Objective::SmoothnessMacro(smooth_macro_obj));
    // vec![
    //     // lively_lib::objectives::objective::Objective::PositionMatch(pos_match_obj),
    //     // lively_lib::objectives::objective::Objective::CollisionAvoidance(col_avoid_obj),
    //     // lively_lib::objectives::objective::Objective::SmoothnessMacro(smooth_macro_obj),
    // ];
    let mut goals: HashMap<String,Goal> = HashMap::new();
    let mut weights: HashMap<String,f64> = HashMap::new();
    goals.insert("iowsdsfhwe".into(),Goal::Translation(Translation3::new(0.5,0.0,0.5)));
    weights.insert("iowsdsfhwe".into(),10.0);

    let mut 
    temp = Solver::new(
        data.clone(),
        objectives.clone(),
        None,//Some(scalar_range_vec.clone()),
        None,
        None,
        None,
        None,
        None
    );
    temp.compute_average_distance_table();
    println!("{:?}",temp.get_current_state());
    

    // let vec = temp.compute_average_distance_table();
    let instant = Instant::now();
    temp.solve(goals, weights, 0.0, None);
    println!("{:?}",instant.elapsed());
}
