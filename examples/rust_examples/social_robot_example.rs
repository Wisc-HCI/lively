use lively::lively::Solver;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::core::base::CollisionAvoidanceObjective;
use lively::objectives::core::base::JointLimitsObjective;
use lively::objectives::core::matching::PositionMatchObjective;
use lively::objectives::core::matching::OrientationMatchObjective;
use lively::objectives::core::matching::JointMatchObjective;
use lively::objectives::liveliness::perlin::OrientationLivelinessObjective;
use lively::objectives::objective::Objective;
use std::collections::HashMap;
use std::fs;

fn main() {
    let mut objectives: HashMap<String, Objective> = HashMap::new();
    // Add a Smoothness Macro Objective
    objectives.insert(
        "smoothness".into(),Objective::SmoothnessMacro(SmoothnessMacroObjective::new("MySmoothnessObjective".to_string(), 25.0, true,false,false))  
    );
    objectives.insert(
        "collision".into(), Objective::CollisionAvoidance(CollisionAvoidanceObjective::new("MyCollisionAvoidanceObjective".to_string(), 0.5))
    );
    objectives.insert(
        "jointLimit".into(), Objective::JointLimits(JointLimitsObjective::new("MyJointLimits".to_string(), 5.0))
    );
    objectives.insert(
        "torsoPosition".into(), Objective::PositionMatch(PositionMatchObjective::new("Torso Position".to_string(), 10.0, "torso".to_string()))
    );
    objectives.insert(
        "RHandPosition".into(), Objective::PositionMatch(PositionMatchObjective::new("R Hand Position".to_string(), 10.0 , "r_gripper".to_string()))
    );
    objectives.insert(
        "RHandOrientation".into(), Objective::OrientationMatch(OrientationMatchObjective::new("R Hand Orientation".to_string(), 10.0 , "r_gripper".to_string()))
    );
    objectives.insert(
        "HeadOrientation".into(), Objective::OrientationMatch(OrientationMatchObjective::new("Gaze".to_string(), 7.0 , "Head".to_string()))
    );

    objectives.insert(
        "Idle Gaze".into(), Objective::OrientationLiveliness(OrientationLivelinessObjective::new("OrientationLiveliness".to_string(), 0.0, "Head".to_string(), 10.0))
    );
    objectives.insert(
        "LShoulderPitch".into(), Objective::JointMatch(JointMatchObjective::new("LShoulderPitch".to_string(),10.0, "LShoulderPitch".to_string()))
    );
    objectives.insert(
        "LShoulderRoll".into(), Objective::JointMatch(JointMatchObjective::new("LShoulderRoll".to_string(),10.0, "LShoulderRoll".to_string()))
    );
    objectives.insert(
        "LElbowYaw".into(), Objective::JointMatch(JointMatchObjective::new("LElbowYaw".to_string(),10.0, "LElbowYaw".to_string()))
    );
    objectives.insert(
        "LElbowRoll".into(), Objective::JointMatch(JointMatchObjective::new("LElbowRoll".to_string(),10.0, "LElbowRoll".to_string()))
    );
    objectives.insert(
        "LWristYaw".into(), Objective::JointMatch(JointMatchObjective::new("LWristYaw".to_string(),10.0, "LWristYaw".to_string()))
    );
    objectives.insert(
        "LHand".into(), Objective::JointMatch(JointMatchObjective::new("LHand".to_string(),10.0, "LHand".to_string()))
    );


    
let data = fs::read_to_string("./tests/pepper.xml").expect("Something went wrong reading the file");
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
