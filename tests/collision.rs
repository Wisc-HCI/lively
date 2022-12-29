use lively::lively::Solver;
use lively::objectives::core::base::CollisionAvoidanceObjective;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::core::matching::PositionMatchObjective;
use lively::objectives::objective::Objective;
use lively::utils::shapes::*;
use lively::utils::info::{*};
use lively::utils::shapes::Shape;
use nalgebra::base::Vector4;
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use nalgebra::geometry::Quaternion;
use nalgebra::geometry::Translation3;
use nalgebra::geometry::UnitQuaternion;

use rand::Rng;

use std::fs;

fn main() {
    let data =
        fs::read_to_string("./tests/ur3e.xml").expect("Something went wrong reading the file");

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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        true,
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
        "world".to_string(),
        true,
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
        true,
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
        "mk2CollisionShapeSpool".to_string(),
        "shoulder_link".to_string(),
        true,
        0.4,
        0.25,
        0.25,
        iso_20,
    ));

    let mut box_shapes_vec: Vec<Shape> = vec![
        box_1, box_2, box_3, box_4, box_5, box_6, box_7, box_8, box_9, box_10, box_11, box_12,
        box_13, box_14, box_15, box_16, box_17.clone(), box_18, box_19.clone(),box_20.clone()
    ];

    for i in 1..= 0{
        
        let mut rng = rand::thread_rng();
        let n : f64 = rng.gen_range(-2.0..2.0);
        let n1 : f64 = rng.gen_range(-2.0..2.0);
        let n2 : f64 = rng.gen_range(-2.0..2.0);
        let temp_translate = Translation3::new(n,n1,n2);
        let n8 : f64 = rng.gen_range(-10.0..10.0);
        let n6 : f64 = rng.gen_range(-10.0..10.0);
        let n7 : f64 = rng.gen_range(-10.0..10.0);
        let temp_quat = UnitQuaternion::new(Vector3::new(n7,n8,n6));
        let iso = Isometry3::from_parts(temp_translate, temp_quat);
        let n3 : usize = rng.gen_range(0..4);
        if n3 == 0{
            let n4 : f64 = rng.gen_range(0.0..1.0);
            let n5 : f64 = rng.gen_range(0.0..1.0);
            let n6 : f64 = rng.gen_range(0.0..1.0);
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

        }else if n3 == 1 {
            let n4 : f64 = rng.gen_range(0.0..1.0);
            let n5 : f64 = rng.gen_range(0.0..1.0);
            
            let temp_cylinder = Shape::Cylinder(CylinderShape::new(
                "shape".to_string(),
                "world".to_string(),
                true,
                n4,
                n5,
                iso,
            ));
            box_shapes_vec.push(temp_cylinder);


        }else if n3 ==2 {
            let n4 : f64 = rng.gen_range(0.0..1.0);
            
            let temp_sphere = Shape::Sphere(SphereShape::new(
                "shape".to_string(),
                "world".to_string(),
                true,
                n4,
                iso,
            ));
            box_shapes_vec.push(temp_sphere);

        }else if n3 == 3{
            let n4 : f64 = rng.gen_range(0.0..1.0);
            let n5 : f64 = rng.gen_range(0.0..1.0);
            
            let temp_capsule= Shape::Capsule(CapsuleShape::new(
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

    let objective_vec: Vec<Objective> = vec![
        Objective::PositionMatch(pos_match_obj),
        Objective::CollisionAvoidance(col_avoid_obj),
        Objective::SmoothnessMacro(smooth_macro_obj),
    ];
    

    let mut 
    temp = Solver::new(
        data.clone(),
        objective_vec.clone(),
        Some(scalar_range_vec.clone()),
        None,
        None,
        None,
        None,
        None,
        None,
    );

    let mut rng = rand::thread_rng();

    let n4 : f64 = rng.gen_range(0.0..1.0);
            let n5 : f64 = rng.gen_range(0.0..1.0);
            let n6 : f64 = rng.gen_range(0.0..1.0);
            let n : f64 = rng.gen_range(-2.0..2.0);
            let n1 : f64 = rng.gen_range(-2.0..2.0);
            let n2 : f64 = rng.gen_range(-2.0..2.0);
            let temp_translate = Translation3::new(n,n1,n2);
            let n8 : f64 = rng.gen_range(-10.0..10.0);
            let n6 : f64 = rng.gen_range(-10.0..10.0);
            let n7 : f64 = rng.gen_range(-10.0..10.0);
            let temp_quat = UnitQuaternion::new(Vector3::new(n7,n8,n6));
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

    let temp_capsule1= Shape::Capsule(CapsuleShape::new(
        "shape".to_string(),
        "world".to_string(),
        true,
        n4,
        n5,
        iso,
    ));

    let temp_capsule2= Shape::Capsule(CapsuleShape::new(
        "shape".to_string(),
        "wrist_2_link".to_string(),
        true,
        n4,
        n5,
        iso,
    ));
    



    let shape_update : Vec<ShapeUpdate> = vec![
        // box
        ShapeUpdate::Add{
            id : 1.to_string(),
            shape : box_19.clone() //"world"
        },
        ShapeUpdate::Add{
            id : 2.to_string(),
            shape : box_20.clone() // existing
        },
        //cylinder
        ShapeUpdate::Add{
            id : "3".to_string(),
            shape : temp_cylinder1.clone() // world
        },

        ShapeUpdate::Add{
            id : "4".to_string(),
            shape : temp_cylinder2.clone() // existing
        },

        //sphere
        ShapeUpdate::Add{
            id : "5".to_string(),
            shape : temp_sphere1.clone() // world
        },

        ShapeUpdate::Add{
            id : "6".to_string(),
            shape : temp_sphere2.clone() // existing
        },

        //capsule
        ShapeUpdate::Add{
            id : "7".to_string(),
            shape : temp_capsule1.clone() // world
        },

        ShapeUpdate::Add{
            id : "8".to_string(),
            shape : temp_capsule2.clone() // existing
        },

        ShapeUpdate::Add{
            id : 1.to_string(),
            shape : box_19.clone() //"world"
        },
        ShapeUpdate::Add{
            id : 2.to_string(),
            shape : box_20.clone() // existing
        },
        //cylinder
        ShapeUpdate::Add{
            id : "3".to_string(),
            shape : temp_cylinder1.clone() // world
        },

        ShapeUpdate::Add{
            id : "4".to_string(),
            shape : temp_cylinder2.clone() // existing
        },

        //sphere
        ShapeUpdate::Add{
            id : "5".to_string(),
            shape : temp_sphere1.clone() // world
        },

        ShapeUpdate::Add{
            id : "6".to_string(),
            shape : temp_sphere2.clone() // existing
        },

        //capsule
        ShapeUpdate::Add{
            id : "7".to_string(),
            shape : temp_capsule1.clone() // world
        },

        ShapeUpdate::Add{
            id : "8".to_string(),
            shape : temp_capsule2.clone() // existing
        },
        //-----------------------------------------------------------------------existing ids
        ShapeUpdate::Add{
            id : 1.to_string(),
            shape : box_19.clone() //"world"
        },
        ShapeUpdate::Add{
            id : 2.to_string(),
            shape : box_20.clone() // existing
        },
        //cylinder
        ShapeUpdate::Add{
            id : "3".to_string(),
            shape : temp_cylinder1.clone() // world
        },

        ShapeUpdate::Add{
            id : "4".to_string(),
            shape : temp_cylinder2.clone() // existing
        },

        //sphere
        ShapeUpdate::Add{
            id : "5".to_string(),
            shape : temp_sphere1.clone() // world
        },

        ShapeUpdate::Add{
            id : "6".to_string(),
            shape : temp_sphere2.clone() // existing
        },

        //capsule
        ShapeUpdate::Add{
            id : "7".to_string(),
            shape : temp_sphere1.clone() // world
        },

        ShapeUpdate::Add{
            id : "8".to_string(),
            shape : temp_sphere2.clone() // existing
        },


        

       
        ShapeUpdate::Move{
            id : "1".to_string(),
            pose : iso_18.clone()
        },
        ShapeUpdate::Move{
            id : "2".to_string(),
            pose : iso_19.clone()
        },
        ShapeUpdate::Move{
            id : "2".to_string(),
            pose : iso_20.clone()
        },

        ShapeUpdate::Move{
            id : "2".to_string(),
            pose : iso_18.clone()
        },

        ShapeUpdate::Move{
            id : "2".to_string(),
            pose : iso_2.clone()
        },

        ShapeUpdate::Move{
            id : "2".to_string(),
            pose : iso_19.clone()
        },

        ShapeUpdate::Move{
            id : "2".to_string(),
            pose : iso_19.clone()
        },

        ShapeUpdate::Move{
            id : "2".to_string(),
            pose : iso_19.clone()
        },

        ShapeUpdate::Move{
            id : "3".to_string(),
            pose : iso_19.clone()
        },
        ShapeUpdate::Move{
            id : "5".to_string(),
            pose : iso_19.clone()
        },
        ShapeUpdate::Move{
            id : "6".to_string(),
            pose : iso_19.clone()
        },
        ShapeUpdate::Move{
            id : "7".to_string(),
            pose : iso_1.clone()
        },
        ShapeUpdate::Move{
            id : "8".to_string(),
            pose : iso_3.clone()
        },
        ShapeUpdate::Move{
            id : "9".to_string(),
            pose : iso_3.clone()
        },






       



    ];
    let vec = temp.compute_average_distance_table();
    let temp_solve = temp.solve(None, None, 0.0, None);



    println!("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    let mut 
    temp2 = Solver::new(
        data,
        objective_vec,
        Some(scalar_range_vec),
        None,
        Some(temp.vars.history.prev1),
        Some(true),
        None,
        None,
        None,
    );

    let temp_solve2 = temp2.solve(None, None, 0.0, None);


    //temp.reset(temp_solve,None);
    //temp.dele
    //temp.perform_updates();
}
