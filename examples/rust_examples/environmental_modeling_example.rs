use lively::lively::Solver;
use lively::objectives::core::base::CollisionAvoidanceObjective;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::objective::Objective;
use lively::utils::info::AddShape;
use lively::utils::info::ShapeUpdate;
use lively::utils::shapes::*;
use nalgebra::geometry::Translation3;
use nalgebra::Isometry3;
use nalgebra::Quaternion;
use nalgebra::UnitQuaternion;
use std::collections::HashMap;
use std::fs;

fn main() {
    let mut objectives: HashMap<String, Objective> = HashMap::new();
    // Add a Smoothness Macro Objective
    objectives.insert(
        "smoothness".into(),
        // An example objective (smoothness macro)
        Objective::SmoothnessMacro(SmoothnessMacroObjective::new(
            "MySmoothnessObjective".to_string(),
            5.0,
            true,
            false,
            false,
        )),
    );
    // Add Collision Avoidance Objective
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
    let mut box_shapes_vec: Vec<Shape> = vec![box_1];
    let data =
        fs::read_to_string("./tests/basic.xml").expect("Something went wrong reading the file");
    let mut solver = Solver::new(
        data.clone(),         // Full urdf as a string
        objectives,           // objectives
        None,                 //root_bounds
        Some(box_shapes_vec), //static environmental shapes
        None,                 // inital_state
        None,                 //max_retries
        None,                 //max_iterations
        None,
    ); //collision_settings

    let iso_2 = Isometry3::from_parts(
        Translation3::new(-1.0, 0.5, 2.1),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            -0.7069999677447771,
            0.7072135784958345,
        )),
    );
    //box_2 here is not a static environmental shape because it will be added to the environment by shape_update
    let box_2 = Shape::Box(BoxShape::new(
        "panda_shapebox".to_string(),
        "world".to_string(),
        true,
        1.5,
        1.7,
        1.0,
        iso_2,
    ));

    // add box_2 to the environment.
    let add_shape_update = AddShape { 
        id: 1.to_string(), //The id must be unique
        shape: box_2.clone(),
    };
    let shape_update: Vec<ShapeUpdate> = vec![ShapeUpdate::Add(add_shape_update)]; // shape_update

    // Run solve to get a solved state
    let state = solver.solve(HashMap::new(), HashMap::new(), 0.0, Some(shape_update));
    // Log the initial state
    println!("{:?}", state);
}
