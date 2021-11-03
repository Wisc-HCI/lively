use std::collections::HashMap;
use crate::utils::shapes::*;
use crate::utils::info::ProximityInfo;
use nalgebra::geometry::{Isometry3};

#[derive(Clone,Debug)]
pub struct CollisionManager {
    persistent_shapes: Vec<Shape>, // Meant to be a recipe for creating rapier objects. Could also serve a way of maintaining references/handles to each of the rapeir shapes themselves.
    robot_shapes: Vec<Shape> // You don't actually have to store the robot shapes as Shapes, but you may if you want to be consistent
    // Feel free to add your other fields here
}

impl CollisionManager {
    pub fn new(persistent_shapes: Vec<Shape>) -> Self {
        // Remove the _ in front of robot to use this parameter.

        let robot_shapes: Vec<Shape> = vec![];
        // enumerate through the robot links and convert the collision objects into rapier stuff.

        Self { persistent_shapes, robot_shapes }
    }

    // pub fn set_robot_frames(&mut self, _frames: &HashMap<String, Isometry3<f64>>) {

    // }

    pub fn set_transient_shapes(&mut self, _transient_shapes: &Vec<Shape>) {

    }

    pub fn get_proximity(&self, _frames: &HashMap<String, Isometry3<f64>>) -> Vec<ProximityInfo> {
        /*
        Generate the vector of proximityInfo structs and return
        */
        return vec![]
    }
}

/*
Frames:
- world frame
- robot frame 1
- robot frame 2
...

Shapes:
- Robot shapes (move with robot frames)
- Static shapes in the world frame (never move)
- 'Static' shapes attached to the robot in robot frames (augmenting the robot)
- Transient shapes attached to the robot (move with robot)
- Transient shapes attached to the world frame

Collisions that matter:
- Collisions between different groups of frames (robot part 1 with part2)
- Collisions between transient shapes and robot/static shapes

Collisions that don't matter:
- Collisions within groups (frames)
- Collisions between transient shapes

Groups (Option 1 - hard to edit groups):
- Group for all static shapes in world frame (Set up at beginning, never update positions)
- Group for each frame of the robot + static shapes attached (Set up at beginning, update based on `set_robot_frames`)
- Group for all transient shapes in world frame (Add/Remove during `set_transient_shapes`)
- Group for each frame of the robot (transient shapes) (Add/Remove during `set_transient_shapes` and move in `set_robot_frames`)

Groups (Option 2 - easy to edit groups):
- Group for all shapes in world frame (Set up at beginning, add/remove shapes to group during `set_transient_shapes`)
- Group for each frame of the robot + static shapes attached (Set up at beginning, add/remove_shapes to group during `set_robot_frames`, update based on `set_robot_frames`)

## Plan for CoFrame (return with score)
[
    {frame1: String, frame2: String, distance: f64}, // Struct
    ...
]

Table
X        Frame1    Frame2   Frame3 ...
Frame1     X         0         0
Frame2     !         X         0
Frame3     !         !         X

*/