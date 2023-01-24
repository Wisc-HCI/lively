//! A simple 3D scene with light shining over a cube sitting on a plane.
#[cfg(feature = "bevy")]
use bevy::{
    prelude::*,
    time::{FixedTimestep},
};

use lively::lively::Solver;
use lively::objectives::core::base::CollisionAvoidanceObjective;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::core::matching::PositionMatchObjective;
use lively::objectives::objective::Objective;
use lively::utils::goals::Goal;
use lively::utils::info::TransformInfo;
use lively::utils::shapes::*;
use nalgebra::geometry::Translation3;
use nalgebra::Isometry3;
#[cfg(feature = "bevy")]
use smooth_bevy_cameras::{
    controllers::orbit::{OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin},
    LookTransformPlugin,
};

use std::collections::HashMap;
use std::fs;

use nalgebra::geometry::Quaternion;

use nalgebra::geometry::UnitQuaternion;
// use bevy_mod_picking::*;

const POSITION_OBJECTIVE: &str = "PositionObjective";
const SMOOTHNESS_OBJECTIVE: &str = "SmoothnessObjective";
const COLLISION_OBJECTIVE: &str = "CollisionObjective";
const TIME_LABEL: &str = "timestep";

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

#[derive(Component, Default)]
pub struct FrameName(String);

#[derive(Component, Default)]
pub struct ObjectiveName(String);

#[derive(Bundle, Default)]
struct LinkBundle {
    #[bundle]
    pbr: PbrBundle,
    frame: FrameName,
}

#[derive(Bundle, Default)]
struct ObjectiveBundle {
    #[bundle]
    pbr: PbrBundle,
    objective: ObjectiveName,
}

#[derive(Component)]
struct Children;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(LookTransformPlugin)
        .add_plugin(OrbitCameraPlugin::default())
        .add_plugin(LivelyTKPlugin)
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .add_plugin(bevy_transform_gizmo::TransformGizmoPlugin::default(
            // Align the gizmo to a different coordinate system.
        )) // Use TransformGizmoPlugin::default() to align to the scene's coordinate system.
        .add_startup_system(setup)
        .run();
}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // plane
    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane { size: 5.0 })),
        transform: Transform::from_rotation(Quat::from_euler(
            EulerRot::XYZ,
            std::f32::consts::PI / 2.0,
            0.0,
            0.0,
        )),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });

    // light
    commands.spawn_bundle(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, -8.0, 4.0),
        ..default()
    });

    commands.insert_resource(AmbientLight {
        color: Color::ANTIQUE_WHITE,
        brightness: 0.3,
    });
    // camera
    // Orbit Controls
    commands
        .spawn_bundle(Camera3dBundle::default())
        .insert_bundle(bevy_mod_picking::PickingCameraBundle::default())
        .insert(bevy_transform_gizmo::GizmoPickSource::default())
        .insert_bundle(OrbitCameraBundle::new(
            OrbitCameraController::default(),
            // Vec3::new(-2.0, 5.0, 5.0),
            // Vec3::new(0., 0., 0.),
            Vec3::new(0.0, -5.0, 4.0),
             Vec3::new(0.0, 0.00, 0.0),
        ));
}

pub fn solve(
    time: Res<Time>,
    mut solver: ResMut<Solver>,
    mut frame_query: Query<
        (&mut FrameName, &mut Transform),
        (With<FrameName>, Without<ObjectiveName>),
    >,
    objective_query: Query<(&ObjectiveName, &Transform), (With<ObjectiveName>, Without<FrameName>)>,
) {
    let mut goals: HashMap<String, Goal> = HashMap::new();
    let mut weights: HashMap<String, f64> = HashMap::new();

    // println!("x {:?}",x);
    for (_objective_name, transform) in &objective_query {
        goals.insert(
            // For now, just insert everything as a position objective
            POSITION_OBJECTIVE.to_string(),
            Goal::Translation(Translation3::new(
                transform.translation.x as f64,
                transform.translation.y as f64,
                transform.translation.z as f64,
            )),
        );
    }

    weights.insert(POSITION_OBJECTIVE.to_string(), 40.0);

    let state = solver.solve(goals, weights, time.seconds_since_startup(), None);
    // Iterate through transforms and update their locations/rotations from the iso
    // println!("goal {:?} {:?}", x, solver.get_goals().get(POSITION_OBJECTIVE.into()));
    // println!("pos {:?} {:?}",Translation3::new(x, y, 0.5),solver.get_current_state().frames.get("tool0".into()).unwrap_or(&TransformInfo::default()).world.translation);
    for (frame_name, mut transform) in &mut frame_query {
        // println!("frame {:?}",frame_name.0);
        let frame_state: Isometry3<f64> = state
            .frames
            .get(&frame_name.0)
            .unwrap_or(&TransformInfo::default())
            .world;

        let translation = Vec3::new(
            frame_state.translation.vector.x as f32,
            frame_state.translation.vector.y as f32,
            frame_state.translation.vector.z as f32,
        );

        let rotation: Quat = Quat::from_xyzw(
            frame_state.rotation.coords[0] as f32,
            frame_state.rotation.coords[1] as f32,
            frame_state.rotation.coords[2] as f32,
            frame_state.rotation.coords[3] as f32,
        );

        transform.translation = translation;

        transform.rotation = rotation;
    }
}

fn setup_lively_tk(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    solver: Res<Solver>,
) {
    // Goal Visualization/Updating
    commands
        .spawn_bundle(ObjectiveBundle {
            pbr: PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Icosphere {
                    radius: 0.1,
                    subdivisions: 32,
                })),
                material: materials.add(Color::rgba(0.1, 0.9, 0.3, 0.5).into()),
                transform: Transform::from_xyz(0.0, 0.5, 0.5),
                ..Default::default()
            },
            objective: ObjectiveName(POSITION_OBJECTIVE.to_string()),
        })
        .insert_bundle(bevy_mod_picking::PickableBundle::default())
        .insert(bevy_transform_gizmo::GizmoTransformable);

    for link in &solver.robot_model.links {
        let id = commands
            .spawn_bundle(LinkBundle {
                pbr: PbrBundle { ..default() },
                frame: FrameName(link.name.to_string()),
            })
            .id();

        for collision in &link.collisions {
            let mut command = commands.get_or_spawn(id);
            let temp: Mesh = lively_tk::utils::shapes::Shape::into(collision.clone());
            let mesh = meshes.add(temp);

            command.insert(Children).with_children(|p| {
                p.spawn_bundle(PbrBundle {
                    mesh: mesh.clone(),
                    transform: collision.clone().get_transform(),
                    material: materials.add(Color::rgb(0.75, 0.75, 0.75).into()),
                    ..default()
                });
            });
        }
    }

    //------------ adding persistent shapes to the scene
    for shape in &solver.robot_model.get_environmental_objects() {
      // println!("{:?}", shape);
        let temp: Mesh = lively_tk::utils::shapes::Shape::into(shape.clone());
        let mesh = meshes.add(temp);

        commands
            .spawn_bundle(LinkBundle {
                pbr: PbrBundle {
                    transform: Transform::default(),
                    //mesh: mesh.clone(),
                    ..default()
                },
                frame: FrameName(shape.clone().get_frame_name()),
                ..default()
            })
            .insert(Children)
            .with_children(|p| {
                p.spawn_bundle(PbrBundle {
                    transform: shape.clone().get_transform(),
                    mesh: mesh.clone(),
                    material: materials.add(StandardMaterial {
                        base_color: Color::ORANGE_RED,
                        emissive: (Color::ORANGE_RED * 2.),
                        ..default()
                    }),
                    ..default()
                });
            });



        //     commands
        // .spawn_bundle(LinkBundle {
        //     pbr: PbrBundle {
        //         transform: Transform::default(),
        //         //mesh: mesh.clone(),
        //         ..default()
        //     },
        //     frame: FrameName(shape.clone().get_frame_name()),
        //     ..default()
        // })
        // .insert(Children)
        // .with_children(|p| {
        //     p.spawn_bundle(PbrBundle {
        //         transform: shape.clone().get_transform(),
        //         mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        //         material: materials.add(StandardMaterial {
        //             base_color: Color::ORANGE_RED,
        //             emissive: (Color::ORANGE_RED * 2.),
        //             ..default()
        //         }),
        //         ..default()
        //     });
        // });

        
    }

   

    
}

pub struct LivelyTKPlugin;

impl Plugin for LivelyTKPlugin {
    fn build(&self, app: &mut App) {
        // Define a livelyTK instance
        let urdf_string =
            fs::read_to_string("./tests/ur3e.xml").expect("Something went wrong reading the file");

        let pos_match_obj =
            PositionMatchObjective::new("EE Position".to_string(), 0.0, "tool0".to_string());
        let col_avoid_obj =
            CollisionAvoidanceObjective::new("Collision Avoidance".to_string(), 20.0);
        let smooth_macro_obj = SmoothnessMacroObjective::new("Smoothness".to_string(), 30.0);
        let root_bounds: Vec<(f64, f64)> = vec![
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
        ];

        let mut objectives: HashMap<String, Objective> = HashMap::new();
        objectives.insert(
            POSITION_OBJECTIVE.to_string(),
            Objective::PositionMatch(pos_match_obj),
        );
        objectives.insert(
            COLLISION_OBJECTIVE.to_string(),
            Objective::CollisionAvoidance(col_avoid_obj),
        );
        objectives.insert(
            SMOOTHNESS_OBJECTIVE.to_string(),
            Objective::SmoothnessMacro(smooth_macro_obj),
        );

        //--------persistent shapes
        let iso_1 = Isometry3::from_parts(
            Translation3::new(
                0.6497281999999998,
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
            "box".to_string(),
            "world".to_string(),
            true,
            0.5,
            0.75,
            0.5,
            iso_1,
        ));

        let iso_2 = Isometry3::from_parts(
            Translation3::new(
                -0.7,
                -0.44972819999999987,
                -0.050000000000000044,
            ),
            UnitQuaternion::from_quaternion(Quaternion::new(
                0.0,
                0.0,
                -0.7069999677447771,
                0.7072135784958345,
            )),
        );

        // let rec_1 = Shape::Box(BoxShape::new(
        //     "rectangle".to_string(),
        //     "world".to_string(),
        //     true,
        //     0.5,
        //     0.75,
        //     0.25,
        //     iso_2,
        // ));
        let sphere_1 = Shape::Sphere(SphereShape::new(
                "sphere".to_string(),
                "world".to_string(),
                true,
                0.25,
                iso_2,
            ));


            let iso_3 = Isometry3::from_parts(
                Translation3::new(
                    -0.20,
                    -0.74972819999999987,
                    0.0,
                ),
                UnitQuaternion::from_quaternion(Quaternion::new(
                    0.0,
                    0.0,
                    0.7069999677447771,
                    0.7072135784958345,
                )),
            );

            let cylinder_1 =  Shape::Cylinder(CylinderShape::new(
                "cylinder".to_string(),
                "world".to_string(),
                true,
                0.5,
                0.25,
                iso_3,
            ));

            let iso_5 = Isometry3::from_parts(
                Translation3::new(
                    -0.20,
                    0.60,
                    0.0,
                ),
                UnitQuaternion::from_quaternion(Quaternion::new(
                    0.0,
                    0.0,
                    -0.7069999677447771,
                    0.7072135784958345,
                )),
            );

            let capsule_1 =  Shape::Capsule(CapsuleShape::new(
                "capsule".to_string(),
                "world".to_string(),
                true,
                0.5,
                0.25,
                iso_5,
            ));


        //--------

        let mut solver = Solver::new(
            urdf_string.clone(),
            objectives.clone(),
            Some(root_bounds),
            Some(vec![box_1,sphere_1,cylinder_1,capsule_1]),
            None,
            Some(true),
            None,
            None,
            None,
        );
        solver.compute_average_distance_table();

        // println!("{:?}",solver.objective_set.objectives);

        let mut goals: HashMap<String, Goal> = HashMap::new();
        let mut weights: HashMap<String, f64> = HashMap::new();
        // println!("x {:?}",x);
        goals.insert(
            POSITION_OBJECTIVE.to_string(),
            Goal::Translation(Translation3::new(0.0, 0.0, 0.5)),
        );
        weights.insert(POSITION_OBJECTIVE.to_string(), 30.0);

        app.insert_resource(solver)
            .add_startup_system(setup_lively_tk)
            .add_stage_after(
                CoreStage::Update,
                FixedUpdateStage,
                SystemStage::parallel()
                    .with_run_criteria(
                        FixedTimestep::steps_per_second(30.0)
                            // labels are optional. they provide a way to access the current
                            // FixedTimestep state from within a system
                            .with_label(TIME_LABEL),
                    )
                    .with_system(solve),
            );
        // .add_system(solve);
    }
}

// fn update_goals_and_weights()
