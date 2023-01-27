use lively::lively::Solver;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::core::base::CollisionAvoidanceObjective;
use lively::utils::info::ScalarRange;
use lively::objectives::objective::Objective;
use std::collections::HashMap;
use lively::utils::shapes::*;
use nalgebra::geometry::Translation3;
use nalgebra::Isometry3;
use nalgebra::Quaternion;
use nalgebra::UnitQuaternion;
use std::fs;

fn main() {
    let mut objectives: HashMap<String, Objective> = HashMap::new();
    let displacement_bounds: Vec<ScalarRange> = vec![
        ScalarRange {value:0.0,delta:0.0},
        ScalarRange {value:0.25,delta:0.0},
        ScalarRange {value:0.0,delta:0.0},
        ScalarRange {value:0.0,delta:0.0},
        ScalarRange {value:0.0,delta:0.0},
        ScalarRange {value:0.0,delta:0.0},
    ];
    // Add a Smoothness Macro Objective
    objectives.insert(
        "smoothness".into(),
        // An example objective (smoothness macro)
        Objective::SmoothnessMacro(SmoothnessMacroObjective::new("MySmoothnessObjective".to_string(), 5.0, true,false,false))
    );
    objectives.insert(
        "collision_avoidance".into(),
        Objective::CollisionAvoidance(CollisionAvoidanceObjective::new(
            "MyCollisionAvoidance".to_string(),
            10.0,
        )),
    );
    let iso_1 = Isometry3::from_parts(
        // defining transform from translation and rotation
        Translation3::new(
            1.7497281999999998,
            -0.24972819999999987,
            0.050000000000000044,
        ),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            -0.7069999677447771,
            0.7072135784958345,
        )),
    );
    //box_1 here is a static environmental shape. This means that box_1 can not be moved or deleted.
    let box_1 = Shape::Box(BoxShape::new(
        //can be 'CylinderShape', 'CapsuleShape', or 'SphereShape'
        "conveyorCollisionShapeBase".to_string(), // name can be arbitrary
        "world".to_string(),                      // frame name
        true,                                     // physical collision
        1.0,                                      // dimension of the box
        1.1,
        1.7,
        iso_1, // local_transform of the box
    ));
    let box_shapes_vec: Vec<Shape> = vec![box_1];
let data = fs::read_to_string("./tests/basic.xml").expect("Something went wrong reading the file");
let solver = Solver::new(
    data.clone(), // Full urdf as a string
    objectives, //objectives
    Some(displacement_bounds), //root_bounds
    Some(box_shapes_vec.clone()), //shapes
    None, //initial_state
    None, //max_retries
    None, //max_iterations
    None);//collision_settings
}
