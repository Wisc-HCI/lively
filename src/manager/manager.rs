use crate::lively::lively::Solver;
use crate::lively::manager::node::Node;
use crate::lively::manager::edge::Edge;
use crate::lively::utils::info::*;
use crate::lively::utils::shapes::*;
use std::collections::HashMap;

pub struct LivelyManager {
    pub solver: Solver,
    pub nodes: HashMap<String,Node>,
    pub edges: HashMap<String,Edge>
}

impl LivelyManager {
    pub fn new(
    urdf: String, 
    root_bounds: Option<Vec<(f64,f64)>>,
    shapes: Option<Vec<Shape>>,
    initial_state: Option<State>,
    max_retries: Option<usize>,
    max_iterations: Option<usize>,
    collision_settings: Option<CollisionSettingInfo>
) -> Self {
        let objectives:HashMap<String,Objective> = HashMap::new();
        let nodes:HashMap<String,Node> = HashMap::new();
        let edges:HashMap<String,Edge> = HashMap::new();
        let solver: Solver = Solver::new(urdf,objectives,root_bounds,shapes,initial_state,max_retries,max_iterations,collision_settings);
        return Self {
            solver,
            nodes,
            edges
        }
    }

    pub fn add_node(node_id:String,name:String) {

    }

    pub fn add_edge(edge_id:String,name:String,parent_id:String,child_id:String) {
        
    }

    pub fn receive_event(event:String) {
        
    }


}