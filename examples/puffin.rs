use lively::lively::Solver;
use lively::objectives::core::base::CollisionAvoidanceObjective;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::core::matching::PositionMatchObjective;
use lively::objectives::objective::Objective;
use lively::utils::goals::Goal;
use lively::utils::goals::Goal::ScalarRange;
use lively::utils::info::*;
use lively::utils::shapes::Shape;
use lively::utils::shapes::*;
use nalgebra::base::Vector3;
use nalgebra::base::Vector4;
use nalgebra::geometry::Isometry3;
use nalgebra::geometry::Quaternion;
use nalgebra::geometry::Translation3;
use nalgebra::geometry::UnitQuaternion;
use num::integer::Roots;
use optimization_engine::matrix_operations::sum;
use rand::Rng;
use std::collections::HashMap;

use std::fs;

use std::time::Instant;

// mod imgui_support;
// use imgui_support::ImguiManager;

// mod renderer;
// use puffin::set_scopes_on;
// use renderer::Renderer;

fn main() {
    // let data =
    //     fs::read_to_string("./tests/ur3e.xml").expect("Something went wrong reading the file");

    let data =
    fs::read_to_string("./tests/panda.xml").expect("Something went wrong reading the file");

    let pos_match_obj =
         PositionMatchObjective::new("EE Position".to_string(), 20.0, "panda_rightfinger".to_string());
       // PositionMatchObjective::new("EE Position".to_string(), 20.0, "LThumb2_link".to_string());
    let col_avoid_obj = CollisionAvoidanceObjective::new("Collision Avoidance".to_string(), 3.0);
    //let smooth_macro_obj = SmoothnessMacroObjective::new("Smoothness".to_string(), 10.0);
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

    let iso_1 = Isometry3::from_parts(
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
    let box_1 = Shape::Box(BoxShape::new(
        "conveyorCollisionShapeBase".to_string(),
        "world".to_string(),
        true,
        1.0,
        1.1,
        1.7,
        iso_1,
    ));

    let iso_2 = Isometry3::from_parts(
        Translation3::new(0.9499698, -0.2499698, 0.050000000000000044),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            -0.7069999677447772,
            0.7072135784958345,
        )),
    );
    let box_2 = Shape::Box(BoxShape::new(
        "conveyorCollisionShapeBelt".to_string(),
        "world".to_string(),
        false,
        0.75,
        0.65,
        0.25,
        iso_2,
    ));

    let iso_3 = Isometry3::from_parts(
        Translation3::new(-0.5500906000000001, -0.25009060000000005, -0.45),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            0.7069999677447771,
            0.7072135784958345,
        )),
    );
    let box_3 = Shape::Box(BoxShape::new(
        "conveyorRecieverCollisionShapeBase".to_string(),
        "world".to_string(),
        false,
        0.75,
        0.25,
        0.7,
        iso_3,
    ));

    let iso_4 = Isometry3::from_parts(
        Translation3::new(-0.59013137, -0.42502567, -0.025000000000000022),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            0.7069999677447772,
            0.7072135784958345,
        )),
    );
    let box_4 = Shape::Box(BoxShape::new(
        "conveyorRecieverCollisionShapeLeftSplit".to_string(),
        "world".to_string(),
        false,
        0.3,
        0.155,
        0.165,
        iso_4,
    ));

    let iso_5 = Isometry3::from_parts(
        Translation3::new(-0.59002567, -0.07513137000000006, -0.025000000000000022),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            0.7069999677447772,
            0.7072135784958345,
        )),
    );
    let box_5 = Shape::Box(BoxShape::new(
        "conveyorRecieverCollisionShapeRightSplit".to_string(),
        "world".to_string(),
        false,
        0.3,
        0.155,
        0.165,
        iso_5,
    ));

    let iso_6 = Isometry3::from_parts(
        Translation3::new(0.6000755, -0.2500755, -0.3),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            -0.7069999677447771,
            0.7072135784958345,
        )),
    );
    let box_6 = Shape::Box(BoxShape::new(
        "conveyorDispatcherCollisionShapeBase".to_string(),
        "world".to_string(),
        false,
        0.75,
        0.35,
        0.9,
        iso_6,
    ));

    let iso_7 = Isometry3::from_parts(
        Translation3::new(0.65000755, -0.07511325000000005, 0.22499999999999998),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            -0.7069999677447771,
            0.7072135784958345,
        )),
    );
    let box_7 = Shape::Box(BoxShape::new(
        "conveyorDispatcherCollisionShapeLeftSplit".to_string(),
        "world".to_string(),
        false,
        0.255,
        0.275,
        0.175,
        iso_7,
    ));

    let iso_8 = Isometry3::from_parts(
        Translation3::new(0.65011325, -0.42500755, 0.22499999999999998),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            -0.7069999677447771,
            0.7072135784958345,
        )),
    );
    let box_8 = Shape::Box(BoxShape::new(
        "conveyorDispatcherCollisionShapeRightSplit".to_string(),
        "world".to_string(),
        false,
        0.29,
        0.275,
        0.175,
        iso_8,
    ));

    let iso_9 = Isometry3::from_parts(
        Translation3::new(0.65011325, -0.42500755, 0.22499999999999998),
        UnitQuaternion::from_quaternion(Quaternion::new(
            0.0,
            0.0,
            -0.7069999677447771,
            0.7072135784958345,
        )),
    );
    let box_9 = Shape::Box(BoxShape::new(
        "conveyorDispatcherCollisionShapeRightSplit".to_string(),
        "world".to_string(),
        false,
        0.29,
        0.275,
        0.175,
        iso_9,
    ));

    let iso_10 = Isometry3::from_parts(
        Translation3::new(0.0, 0.36, -0.010000000000000009),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_10 = Shape::Box(BoxShape::new(
        "tableCollisionShapeTop".to_string(),
        "world".to_string(),
        false,
        0.1225,
        0.625,
        0.05,
        iso_10,
    ));

    let iso_11 = Isometry3::from_parts(
        Translation3::new(-0.585, 0.07, -0.395),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_11 = Shape::Box(BoxShape::new(
        "tableCollisionShapeFrontLeftLeg".to_string(),
        "world".to_string(),
        false,
        0.05,
        0.05,
        0.75,
        iso_11,
    ));

    let iso_12 = Isometry3::from_parts(
        Translation3::new(-0.585, 0.6499999999999999, -0.585),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_12 = Shape::Box(BoxShape::new(
        "tableCollisionShapeRearLeftLeg".to_string(),
        "world".to_string(),
        false,
        0.05,
        0.05,
        0.75,
        iso_12,
    ));

    let iso_13 = Isometry3::from_parts(
        Translation3::new(0.585, 0.7, -0.395),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_13 = Shape::Box(BoxShape::new(
        "tableCollisionShapeRearRightLeg".to_string(),
        "world".to_string(),
        false,
        0.05,
        0.05,
        0.75,
        iso_13,
    ));

    let iso_14 = Isometry3::from_parts(
        Translation3::new(0.0, -0.15, -0.7150000000000001),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_14 = Shape::Box(BoxShape::new(
        "pedestalCollisionShapeBase".to_string(),
        "world".to_string(),
        false,
        0.65,
        0.65,
        0.15,
        iso_14,
    ));

    let iso_15 = Isometry3::from_parts(
        Translation3::new(0.0, -0.15, -0.33),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_15 = Shape::Box(BoxShape::new(
        "pedestalCollisionShapeTower".to_string(),
        "world".to_string(),
        false,
        0.1,
        0.1,
        0.7,
        iso_15,
    ));

    let iso_16 = Isometry3::from_parts(
        Translation3::new(-0.46, 0.42000000000000004, 0.22500000000000003),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_16 = Shape::Box(BoxShape::new(
        "mk2CollisionShapeLeftVertical".to_string(),
        "world".to_string(),
        false,
        0.125,
        0.185,
        0.4,
        iso_16,
    ));

    let iso_17 = Isometry3::from_parts(
        Translation3::new(-0.10000000000000003, 0.445, 0.22500000000000003),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_17 = Shape::Box(BoxShape::new(
        "mk2CollisionShapeRightVertical".to_string(),
        "world".to_string(),
        false,
        0.125,
        0.225,
        0.4,
        iso_17,
    ));

    let iso_18 = Isometry3::from_parts(
        Translation3::new(-0.28, 0.32, 0.050000000000000044),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_18 = Shape::Box(BoxShape::new(
        "mk2CollisionShapeBase".to_string(),
        "upper_arm_link".to_string(),
        false,
        0.4,
        0.4,
        0.1,
        iso_18,
    ));

    let iso_19 = Isometry3::from_parts(
        Translation3::new(-0.28, 0.445, 0.48000000000000004),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_19 = Shape::Box(BoxShape::new(
        "mk2CollisionShapeSpool".to_string(),
        "world".to_string(),
        false,
        0.4,
        0.25,
        0.25,
        iso_19,
    ));

    let iso_20 = Isometry3::from_parts(
        Translation3::new(-0.28, 0.445, 0.48000000000000004),
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
    );
    let box_20 = Shape::Box(BoxShape::new(
        "mk2CollisionShapeSpool1".to_string(),
        "shoulder_link".to_string(),
        false,
        0.4,
        0.25,
        0.25,
        iso_20.clone(),
    ));

    let eroor_box = Shape::Box(BoxShape::new(
        "error box".to_string(),
        "not exist".to_string(),
        false,
        0.4,
        0.25,
        0.25,
        iso_20,
    ));

    let mut box_shapes_vec: Vec<Shape> = vec![
        // box_1,
        // box_2,
        // box_3,
        // box_4,
        // box_5,
        // box_6,
        // box_7,
        // box_8,
        // box_9,
        // box_10,
        // box_11,
        // box_12,
        // box_13,
        // box_14,
        // box_15,
        // box_16.clone(),
        // box_17.clone(),
        // box_18.clone(),
        // box_19.clone(),
        // box_20.clone(),
    ];



    let mut objectives: HashMap<String, Objective> = HashMap::new();
    objectives.insert("iowsdsfhwe".into(), Objective::PositionMatch(pos_match_obj));
    objectives.insert(
        "sdfsddsfes".into(),
        Objective::CollisionAvoidance(col_avoid_obj),
    );
    // objectives.insert(
    //     "dfawdaseas".into(),
    //     Objective::SmoothnessMacro(smooth_macro_obj),
    // );

   

    let mut rng = rand::thread_rng();

    let n4: f64 = rng.gen_range(0.0..1.0);
    let n5: f64 = rng.gen_range(0.0..1.0);
    let n6: f64 = rng.gen_range(0.0..1.0);
    let n: f64 = rng.gen_range(-2.0..2.0);
    let n1: f64 = rng.gen_range(-2.0..2.0);
    let n2: f64 = rng.gen_range(-2.0..2.0);
    let temp_translate = Translation3::new(n, n1, n2);
    let n8: f64 = rng.gen_range(-10.0..10.0);
    let n6: f64 = rng.gen_range(-10.0..10.0);
    let n7: f64 = rng.gen_range(-10.0..10.0);
    let temp_quat = UnitQuaternion::new(Vector3::new(n7, n8, n6));
    let iso = Isometry3::from_parts(temp_translate, temp_quat);

    let temp_cylinder1 = Shape::Cylinder(CylinderShape::new(
        "shape".to_string(),
        "world".to_string(),
        true,
        n4,
        n5,
        iso,
    ));

    let temp_cylinder2 = Shape::Cylinder(CylinderShape::new(
        "shape".to_string(),
        "upper_arm_link".to_string(),
        true,
        n4,
        n5,
        iso,
    ));

    let temp_sphere1 = Shape::Sphere(SphereShape::new(
        "shape".to_string(),
        "world".to_string(),
        true,
        n4,
        iso,
    ));

    let temp_sphere2 = Shape::Sphere(SphereShape::new(
        "shape".to_string(),
        "wrist_1_link".to_string(),
        true,
        n4,
        iso,
    ));

    let temp_capsule1 = Shape::Capsule(CapsuleShape::new(
        "shape".to_string(),
        "world".to_string(),
        true,
        n4,
        n5,
        iso,
    ));

    let temp_capsule2 = Shape::Capsule(CapsuleShape::new(
        "shape".to_string(),
        "wrist_2_link".to_string(),
        true,
        n4,
        n5,
        iso,
    ));

    let shape_update: Vec<ShapeUpdate> = vec![
        // box
        ShapeUpdate::Add {
            id: 1.to_string(),
            shape: box_19.clone(), //new "world"
        },
        ShapeUpdate::Add {
            id: 2.to_string(),
            shape: box_20.clone(), //new robot frame
        },
        // box
        ShapeUpdate::Add {
            id: 1.to_string(),
            shape: box_20.clone(), //new "world"
        },
        ShapeUpdate::Add {
            id: 2.to_string(),
            shape: box_19.clone(), //new robot frame
        },
        ShapeUpdate::Move {
            id: 1.to_string(),
            pose: iso_20.clone(), //new robot frame
        },
        ShapeUpdate::Move {
            id: 2.to_string(),
            pose: iso_19.clone(), //new robot frame
        },
        ShapeUpdate::Move {
            id: 3.to_string(),
            pose: iso_20.clone(), //new robot frame
        },
        ShapeUpdate::Move {
            id: 4.to_string(),
            pose: iso_19.clone(), //new robot frame
        },
        // ShapeUpdate::Delete(2.to_string()),
        // ShapeUpdate::Delete(1.to_string()),
        // ShapeUpdate::Delete(3.to_string()),
        // ShapeUpdate::Delete(4.to_string())

        // ShapeUpdate::Add{
        //     id : 1.to_string(),
        //     shape : box_16.clone() //replace world to world
        // },

        // ShapeUpdate::Add{
        //     id : 1.to_string(),
        //     shape : box_18.clone() //replace world to robot frame
        // },

        // ShapeUpdate::Add{
        //     id : 2.to_string(),
        //     shape : box_18.clone() //replace robot frame to robot frame
        // },

        // ShapeUpdate::Add{
        //     id : 2.to_string(),
        //     shape : box_16.clone() //replace robot frame to world
        // },

        // ShapeUpdate::Add{
        //     id : 2.to_string(),
        //     shape : box_16.clone() //replace robot frame to world
        // },

        // ShapeUpdate::Add{
        //     id : 2.to_string(),
        //     shape : box_16.clone() //replace robot frame to world
        // },

        // ShapeUpdate::Add{
        //     id : 10.to_string(),
        //     shape : eroor_box.clone() //error frame
        // },

        // ShapeUpdate::Add{
        //     id : 1.to_string(),
        //     shape : eroor_box.clone() //error frame
        // },

        // ShapeUpdate::Add{
        //     id : 2.to_string(),
        //     shape : eroor_box.clone() //error frame
        // },

        //cylinder
        // ShapeUpdate::Add{
        //     id : "3".to_string(),
        //     shape : temp_cylinder1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "4".to_string(),
        //     shape : temp_cylinder2.clone() // existing
        // },

        // ShapeUpdate::Add{
        //     id : "3".to_string(),
        //     shape : temp_cylinder1.clone() // replace world to world
        // },

        // ShapeUpdate::Add{
        //     id : "3".to_string(),
        //     shape : temp_cylinder2.clone() // replace world to robot frame
        // },

        // ShapeUpdate::Add{
        //     id : "4".to_string(),
        //     shape : temp_cylinder1.clone() // replace robot frame to world
        // },

        // ShapeUpdate::Add{
        //     id : "4".to_string(),
        //     shape : temp_cylinder2.clone() // replace robot frame to robot frame
        // },

        // ShapeUpdate::Add{
        //     id : 11.to_string(),
        //     shape : eroor_box.clone() //error frame
        // },

        // ShapeUpdate::Add{
        //     id : 3.to_string(),
        //     shape : eroor_box.clone() //error frame
        // },

        // ShapeUpdate::Add{
        //     id : 4.to_string(),
        //     shape : eroor_box.clone() //error frame
        // },

        // box

        // //sphere
        // ShapeUpdate::Add{
        //     id : "5".to_string(),
        //     shape : temp_sphere1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "6".to_string(),
        //     shape : temp_sphere2.clone() // existing
        // },

        // //capsule
        // ShapeUpdate::Add{
        //     id : "7".to_string(),
        //     shape : temp_capsule1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "8".to_string(),
        //     shape : temp_capsule2.clone() // existing
        // },

        // ShapeUpdate::Add{
        //     id : 1.to_string(),
        //     shape : box_19.clone() //"world"
        // },
        // ShapeUpdate::Add{
        //     id : 2.to_string(),
        //     shape : box_20.clone() // existing
        // },
        // //cylinder
        // ShapeUpdate::Add{
        //     id : "3".to_string(),
        //     shape : temp_cylinder1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "4".to_string(),
        //     shape : temp_cylinder2.clone() // existing
        // },

        // //sphere
        // ShapeUpdate::Add{
        //     id : "5".to_string(),
        //     shape : temp_sphere1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "6".to_string(),
        //     shape : temp_sphere2.clone() // existing
        // },

        // //capsule
        // ShapeUpdate::Add{
        //     id : "7".to_string(),
        //     shape : temp_capsule1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "8".to_string(),
        //     shape : temp_capsule2.clone() // existing
        // },
        // //-----------------------------------------------------------------------existing ids
        // ShapeUpdate::Add{
        //     id : 1.to_string(),
        //     shape : box_19.clone() //"world"
        // },
        // ShapeUpdate::Add{
        //     id : 2.to_string(),
        //     shape : box_20.clone() // existing
        // },
        // //cylinder
        // ShapeUpdate::Add{
        //     id : "3".to_string(),
        //     shape : temp_cylinder1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "4".to_string(),
        //     shape : temp_cylinder2.clone() // existing
        // },

        // //sphere
        // ShapeUpdate::Add{
        //     id : "5".to_string(),
        //     shape : temp_sphere1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "6".to_string(),
        //     shape : temp_sphere2.clone() // existing
        // },

        // //capsule
        // ShapeUpdate::Add{
        //     id : "7".to_string(),
        //     shape : temp_sphere1.clone() // world
        // },

        // ShapeUpdate::Add{
        //     id : "8".to_string(),
        //     shape : temp_sphere2.clone() // existing
        // },

        // ShapeUpdate::Move{
        //     id : "1".to_string(),
        //     pose : iso_18.clone()
        // },
        // ShapeUpdate::Move{
        //     id : "2".to_string(),
        //     pose : iso_19.clone()
        // },
        // ShapeUpdate::Move{
        //     id : "2".to_string(),
        //     pose : iso_20.clone()
        // },

        // ShapeUpdate::Move{
        //     id : "2".to_string(),
        //     pose : iso_18.clone()
        // },

        // ShapeUpdate::Move{
        //     id : "2".to_string(),
        //     pose : iso_2.clone()
        // },

        // ShapeUpdate::Move{
        //     id : "2".to_string(),
        //     pose : iso_19.clone()
        // },

        // ShapeUpdate::Move{
        //     id : "2".to_string(),
        //     pose : iso_19.clone()
        // },

        // ShapeUpdate::Move{
        //     id : "2".to_string(),
        //     pose : iso_19.clone()
        // },

        // ShapeUpdate::Move{
        //     id : "3".to_string(),
        //     pose : iso_19.clone()
        // },
        // ShapeUpdate::Move{
        //     id : "5".to_string(),
        //     pose : iso_19.clone()
        // },
        // ShapeUpdate::Move{
        //     id : "6".to_string(),
        //     pose : iso_19.clone()
        // },
        // ShapeUpdate::Move{
        //     id : "7".to_string(),
        //     pose : iso_1.clone()
        // },
        // ShapeUpdate::Move{
        //     id : "8".to_string(),
        //     pose : iso_3.clone()
        // },
        // ShapeUpdate::Move{
        //     id : "9".to_string(),
        //     pose : iso_3.clone()
        // },
    ];

    let mut goals: HashMap<String, Goal> = HashMap::new();
    let mut weights: HashMap<String, f64> = HashMap::new();
    goals.insert(
        "iowsdsfhwe".into(),
        Goal::Translation(Translation3::new(0.5, 0.0, 0.5)),
    );
    weights.insert("iowsdsfhwe".into(), 10.0);

    // let vec = temp.compute_average_distance_table();
    println!("{:?}" , box_shapes_vec.len());

    
    let mut times: Vec<instant::Duration> = vec![];
    for i in 1..= 100 {
        for i in 1..= 60 {
            let mut rng = rand::thread_rng();
            let n: f64 = rng.gen_range(-2.0..2.0);
            let n1: f64 = rng.gen_range(-2.0..2.0);
            let n2: f64 = rng.gen_range(-2.0..2.0);
            let temp_translate = Translation3::new(n, n1, n2);
            let n8: f64 = rng.gen_range(-10.0..10.0);
            let n6: f64 = rng.gen_range(-10.0..10.0);
            let n7: f64 = rng.gen_range(-10.0..10.0);
            let temp_quat = UnitQuaternion::new(Vector3::new(n7, n8, n6));
            let iso = Isometry3::from_parts(temp_translate, temp_quat);
            let n3: usize = rng.gen_range(0..4);
            if n3 == 0 {
                let n4: f64 = rng.gen_range(0.0..1.0);
                let n5: f64 = rng.gen_range(0.0..1.0);
                let n6: f64 = rng.gen_range(0.0..1.0);
                let temp_box = Shape::Box(BoxShape::new(
                    "shape".to_string(),
                    "world".to_string(),
                    true,
                    n4,
                    n5,
                    n6,
                    iso,
                ));
                box_shapes_vec.push(temp_box);
            } else if n3 == 1 {
                let n4: f64 = rng.gen_range(0.0..1.0);
                let n5: f64 = rng.gen_range(0.0..1.0);
    
                let temp_cylinder = Shape::Cylinder(CylinderShape::new(
                    "shape".to_string(),
                    "world".to_string(),
                    true,
                    n4,
                    n5,
                    iso,
                ));
                box_shapes_vec.push(temp_cylinder);
            } else if n3 == 2 {
                let n4: f64 = rng.gen_range(0.0..1.0);
    
                let temp_sphere = Shape::Sphere(SphereShape::new(
                    "shape".to_string(),
                    "world".to_string(),
                    true,
                    n4,
                    iso,
                ));
                box_shapes_vec.push(temp_sphere);
            } else if n3 == 3 {
                let n4: f64 = rng.gen_range(0.0..1.0);
                let n5: f64 = rng.gen_range(0.0..1.0);
    
                let temp_capsule = Shape::Capsule(CapsuleShape::new(
                    "shape".to_string(),
                    "world".to_string(),
                    true,
                    n4,
                    n5,
                    iso,
                ));
                box_shapes_vec.push(temp_capsule);
            }
        }

        let mut solver = Solver::new(
            data.clone(),
            objectives.clone(),
            Some(scalar_range_vec.clone()),
            
            Some(box_shapes_vec.clone()),
            None,
            None,
            None,
            None
        );
        solver.compute_average_distance_table();
       
        for j in 1..= 100 {
            solver.solve(
                goals.clone(),
                weights.clone(),
                0.0,
                //Some(shape_update.clone()
                None,
            );
        }
        let mut total = instant::Duration::new(0,0);
        for j in 1..= 100 { // 0 env shapes
        
            let instant = Instant::now();
            solver.solve(
                goals.clone(),
                weights.clone(),
                0.0,
                //Some(shape_update.clone()
                None,
            );
            total += instant.elapsed();
            
        }
        times.push(total/100);
       

       
        

    }
    
    let mut total = *times.get(0).unwrap();
    for i in 1..= times.len()-1 {
        let time = total.checked_add(*times.get(i).unwrap()) ;
        match time {
            Some(time) => {
                total = time;
            }
            None => {}
        }
    }




    let average = total.checked_div(times.len().try_into().unwrap()).unwrap();
   

    let mut sum_time = 0.0;
    for i in 0..= times.len()-1 {
        let time = *times.get(i).unwrap();
        //print!(" {:?} " , time);
        let mut difference_in_time;
        if time > average {
            difference_in_time = time - average;
        }else{
            difference_in_time = average - time;
        }
       
        //println!(" {:?},{:?},{:?} " , time,average,difference_in_time);
        sum_time += difference_in_time.as_secs_f64() * difference_in_time.as_secs_f64();
    }
   
    // let sum_time_duration = instant::Duration::from_secs_f64(sum_time);
    let div_sum_time = sum_time / (times.len()-1 )as f64;
   //println!("{:?}" , div_sum_time);
    let sd = div_sum_time.sqrt();
    println!("{:?}" , sd);

    let sd_duration = instant::Duration::from_secs_f64(sd);



   println!("the average is {:?}, and sd is {:?} " , average,sd_duration );
    
   
   
   
    

    

     // println!("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");


     //println!("{:?}", instant.elapsed());

     // println!("{:?}", instant.elapsed());
       
        //temp.solve(None, None, 0.0, Some(shape_update));
       // println!("{:?}", instant.elapsed());

    // for item in temp_solve.proximity {
    //     println!("the result getting from temp_solve is {:?}" , item);
    // }

    // println!("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    // let mut
    // temp2 = Solver::new(
    //     data,
    //     objective_vec,
    //     Some(scalar_range_vec),
    //     None,
    //     Some(temp.vars.history.prev1),
    //     Some(true),
    //     None,
    //     None,
    //     None,
    // );

    // let temp_solve2 = temp2.solve(None, None, 0.0, None);

    //temp.reset(temp_solve,None);
    //temp.dele
    //temp.perform_updates();
}
