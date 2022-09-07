//! A simple 3D scene with light shining over a cube sitting on a plane.

use bevy::{prelude::*,time::{FixedTimestep,FixedTimesteps}};
use bevy_transform_gizmo::TransformGizmoPlugin;
use lively_tk::lively_tk::Solver;
use lively_tk::objectives::core::base::CollisionAvoidanceObjective;
use lively_tk::objectives::core::base::SmoothnessMacroObjective;
use lively_tk::objectives::core::matching::PositionMatchObjective;
use lively_tk::objectives::objective::Objective;
use lively_tk::utils::goals::Goal;
use lively_tk::utils::info::TransformInfo;
use lively_tk::utils::shapes::Shape;
use nalgebra::geometry::Translation3;
use nalgebra::Isometry3;
use smooth_bevy_cameras::{
    controllers::orbit::{OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin},
    LookTransformPlugin,
};
use std::collections::HashMap;
use std::fs;
use bevy_mod_picking::*;

const POSITION_OBJECTIVE: &str = "PositionObjective";
const SMOOTHNESS_OBJECTIVE: &str = "SmoothnessObjective";
const COLLISION_OBJECTIVE: &str = "CollisionObjective";
const TIME_LABEL: &str = "timestep";

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
struct FixedUpdateStage;

#[derive(Component, Default)]
pub struct FrameName(String);

#[derive(Bundle, Default)]
struct LinkBundle {
    #[bundle]
    pbr: PbrBundle,
    frame: FrameName,
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
    // LivelyTK setup
    commands
    .spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1 })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        ..Default::default()
    })
    .insert_bundle(bevy_mod_picking::PickableBundle::default())
    .insert(bevy_transform_gizmo::GizmoTransformable);
    // plane
    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane { size: 5.0 })),
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
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
    // camera
    // Orbit Controls
    commands
        .spawn_bundle(Camera3dBundle::default())
        .insert_bundle(bevy_mod_picking::PickingCameraBundle::default())
        .insert(bevy_transform_gizmo::GizmoPickSource::default())
        .insert_bundle(OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(-2.0, 5.0, 5.0),
            Vec3::new(0., 0., 0.),
        ));
}

pub fn solve(
    time: Res<Time>,
    mut solver: ResMut<Solver>,
    mut query: Query<(&mut FrameName, &mut Transform)>
) {
    let mut goals: HashMap<String, Goal> = HashMap::new();
    let mut weights: HashMap<String, f64> = HashMap::new();
    let x = (time.seconds_since_startup()/10.0).sin()/2.0;
    let y = (time.seconds_since_startup()/10.0).cos()/2.0;
    // println!("x {:?}",x);
    goals.insert(
        POSITION_OBJECTIVE.to_string(),
        Goal::Translation(Translation3::new(x, y, 0.5)),
    );
    weights.insert(POSITION_OBJECTIVE.to_string(), 30.0);
    
    let state = solver.solve(
        goals, 
        weights, 
        time.seconds_since_startup(), 
        None
    );
    // Iterate through transforms and update their locations/rotations from the iso
    // println!("goal {:?} {:?}", x, solver.get_goals().get(POSITION_OBJECTIVE.into()));
    // println!("pos {:?} {:?}",Translation3::new(x, y, 0.5),solver.get_current_state().frames.get("tool0".into()).unwrap_or(&TransformInfo::default()).world.translation);
    for (frame_name, mut transform) in &mut query {
        
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
}

pub struct LivelyTKPlugin;

impl Plugin for LivelyTKPlugin {
    fn build(&self, app: &mut App) {
        // Define a livelyTK instance
        let urdf_string =
            fs::read_to_string("./tests/ur3e.xml").expect("Something went wrong reading the file");

        let pos_match_obj =
            PositionMatchObjective::new("EE Position".to_string(), 40.0, "tool0".to_string());
        let col_avoid_obj =
            CollisionAvoidanceObjective::new("Collision Avoidance".to_string(), 0.00000000001);
        let smooth_macro_obj = SmoothnessMacroObjective::new("Smoothness".to_string(), 15.0);
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
            Objective::PositionMatch(pos_match_obj)
        );
        objectives.insert(
            COLLISION_OBJECTIVE.to_string(),
            Objective::CollisionAvoidance(col_avoid_obj),
        );
        objectives.insert(
            SMOOTHNESS_OBJECTIVE.to_string(),
            Objective::SmoothnessMacro(smooth_macro_obj),
        );

        let mut solver = Solver::new(
            urdf_string.clone(),
            objectives.clone(),
            Some(root_bounds),
            None,
            None,
            Some(true),
            None,
            None,
            None,
        );
        solver.compute_average_distance_table();

        println!("{:?}",solver.objective_set.objectives);

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
                        FixedTimestep::steps_per_second(40.0)
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