use lively::lively::Solver;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::objective::Objective;
use std::collections::HashMap;
use std::fs;

fn main() {
    let mut objectives: HashMap<String, Objective> = HashMap::new();
    // Add a Smoothness Macro Objective
    objectives.insert(
        "smoothness".into(),
        // An example objective (smoothness macro)
        Objective::SmoothnessMacro(SmoothnessMacroObjective::new("MySmoothnessObjective".to_string(), 5.0, true,false,false))
    );
let data = fs::read_to_string("./tests/basic.xml").expect("Something went wrong reading the file");
let mut solver = Solver::new(
    data.clone(), // Full urdf as a string
    objectives, //objectives
    None, //root_bounds
    None, //shapes
    None, //initial_state
    None, //max_retries
    None, //max_iterations
    None); //collision_settings
  // Run solve to get a solved state
  let state = solver.solve(
    HashMap::new(), // empty goals hashmap
    HashMap::new(), // empty weights hashmap
    0.0, // time
    None //shape_update
  );
  // Log the initial state
  println!("{:?}",state);
}
