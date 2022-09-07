//! A simple 3D scene with light shining over a cube sitting on a plane.

use bevy::prelude::*;
use lively_tk::lively_tk::Solver;
use lively_tk::objectives::core::base::CollisionAvoidanceObjective;
use lively_tk::objectives::core::base::SmoothnessMacroObjective;
use lively_tk::objectives::core::matching::PositionMatchObjective;
use lively_tk::objectives::objective::Objective;
use lively_tk::utils::goals::Goal;
use smooth_bevy_cameras::{
    controllers::orbit::{OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin},
    LookTransformPlugin,
};
use std::fs;
use std::collections::HashMap;
use nalgebra::geometry::Translation3;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(LookTransformPlugin)
        .add_plugin(OrbitCameraPlugin::default())
        .add_plugin(LivelyTKPlugin)
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

    // plane
    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane { size: 5.0 })),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });
    // cube
    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
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
        .insert_bundle(OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(-2.0, 5.0, 5.0),
            Vec3::new(0., 0., 0.),
        ));
}

pub fn solve(time: Res<Time>, mut solver: ResMut<Solver>) {
    let mut goals: HashMap<String, Goal> = HashMap::new();
    let mut weights: HashMap<String, f64> = HashMap::new();
    goals.insert(
        "iowsdsfhwe".into(),
        Goal::Translation(Translation3::new(0.5, 0.0, 0.5)),
    );
    weights.insert("iowsdsfhwe".into(), 10.0);
    // This runs the solver. Right now this has no effect, but it will shortly
    solver.solve(goals, weights, time.seconds_since_startup(), None);
}

pub struct LivelyTKPlugin;

impl Plugin for LivelyTKPlugin {
    fn build(&self, app: &mut App) {
        
        // Define a livelyTK instance
        let urdf_string =
            fs::read_to_string("./tests/ur3e.xml").expect("Something went wrong reading the file");

        let pos_match_obj = PositionMatchObjective::new(
            "EE Position".to_string(),
            20.0,
            "tool0".to_string(),
        );
        let col_avoid_obj =
            CollisionAvoidanceObjective::new("Collision Avoidance".to_string(), 10.0);
        let smooth_macro_obj = SmoothnessMacroObjective::new("Smoothness".to_string(), 10.0);
        let root_bounds: Vec<(f64, f64)> = vec![
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0)
        ];

        let mut objectives: HashMap<String, Objective> = HashMap::new();
        objectives.insert("iowsdsfhwe".into(), Objective::PositionMatch(pos_match_obj));
        objectives.insert(
            "sdfsddsfes".into(),
            Objective::CollisionAvoidance(col_avoid_obj),
        );
        objectives.insert(
            "dfawdaseas".into(),
            Objective::SmoothnessMacro(smooth_macro_obj),
        );
        

        let mut solver = Solver::new(
            urdf_string.clone(),
            objectives.clone(),
            Some(root_bounds),
            None,
            None,
            Some(false),
            None,
            None,
            None,
        );
        solver.compute_average_distance_table();

        app.insert_resource(solver)
            // .add_startup_system(add_people)
            .add_system(solve);
    }
}
