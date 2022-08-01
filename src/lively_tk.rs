use optimization_engine::{constraints::*, panoc::*, *};
use crate::utils::info::{*};
use crate::utils::vars::{*};
use crate::utils::shapes::{*};
use crate::utils::robot_model::RobotModel;
use crate::utils::state::State;
use crate::utils::goals::Goal;
use crate::utils::objective_set::ObjectiveSet;
use crate::objectives::objective::Objective;
use rand::{thread_rng, Rng};
use rand::rngs::ThreadRng;
use std::f64::consts::{PI};

pub struct Solver {
    pub robot_model: RobotModel,

    // Optimization utility
    pub vars: Vars,
    pub panoc_cache: PANOCCache,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,
    pub objective_set: ObjectiveSet,

    // Optimization values
    pub xopt: Vec<f64>,
    pub xopt_core: Vec<f64>,

    // Optimization Settings
    pub only_core: bool,
    pub max_retries: usize,
    pub max_iterations: usize,
}

impl Solver {
    pub fn new(
        urdf: String, 
        objectives: Vec<Objective>, 
        root_bounds: Option<Vec<(f64,f64)>>,
        shapes: Option<Vec<Shape>>,
        initial_state: Option<State>,
        only_core: Option<bool>,
        max_retries: Option<usize>,
        max_iterations: Option<usize>
    ) -> Self {
        
        // Define the robot model, which is used for kinematics and handling collisions
        let robot_model = RobotModel::new(urdf, shapes.unwrap_or(vec![]));
        let current_state: State;
        match initial_state {
            Some(state) => current_state = robot_model.get_filled_state(state),
            None => current_state = robot_model.get_default_state()
        }
        robot_model.collision_manager.lock().unwrap().compute_ground_truth_distance_grid(&current_state.frames);
        // Vars contains the variables that are passed along to each objective each solve
        let vars = Vars::new(&current_state, robot_model.joints.clone(), robot_model.links.clone());

        // Panoc_Cache is a PANOCCache
        let panoc_cache = PANOCCache::new(robot_model.dims.clone(), 1e-14, 10);
        // Bounds is defined by the robot root bounds and the joint limits
        let mut lower_bounds: Vec<f64>;
        let mut upper_bounds: Vec<f64>;
        
        match root_bounds {
            Some(bounds) => {
                lower_bounds = Vec::new();
                upper_bounds = Vec::new();
                for bound in bounds {
                    lower_bounds.push(bound.0 + bound.1);
                    upper_bounds.push(bound.0 - bound.1);
                }
            },
            None => {
                lower_bounds = vec![
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                    PI/-2.0,
                    PI/-2.0,
                    PI/-2.0];
                upper_bounds = vec![
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::INFINITY,
                    PI/2.0,
                    PI/2.0,
                    PI/2.0];
            }
        }
        for joint in &robot_model.joints {
            // Only include non-mimic joints
            match joint.mimic {
                Some(_) => {},
                None => {
                    lower_bounds.push(joint.lower_bound.clone());
                    upper_bounds.push(joint.upper_bound.clone());
                }
            }
            
        }

        let initial_x = robot_model.get_x(current_state);
        
        Self {
            robot_model,
            // Non-visible values
            vars,
            panoc_cache,
            objective_set: ObjectiveSet::new(&objectives),
            upper_bounds,
            lower_bounds,
            xopt: initial_x.clone(),
            xopt_core: initial_x.clone(),
            only_core: only_core.unwrap_or(false),
            max_retries: max_retries.unwrap_or(1),
            max_iterations: max_iterations.unwrap_or(150)
        }
    }

    pub fn reset(
        &mut self, 
        state: State,
        weights: Option<Vec<Option<f64>>>
    ) -> () {
        // Hande the state updates
        // First update the robot model, and then use that for updating rest
        let current_state = self.robot_model.get_filled_state(state);
        
        // Handle updating the values in vars
        self.vars.history.reset(&current_state);
        self.vars.history_core.reset(&current_state);
        self.robot_model.collision_manager.lock().unwrap().clear_all_transient_shapes();

        // Handle updating weights
        match weights {
            Some(new_weight_set) => {
                for i in 0..new_weight_set.len() {
                    match new_weight_set[i] {
                        Some(new_weight) => {
                            self.objective_set.objectives[i].set_weight(new_weight)
                        },
                        None => {}
                    }
                }
            },
            None => {}
        }

        ()
    }
    
    #[profiling::function]
    pub fn solve(
        &mut self,
        goals: Option<Vec<Option<Goal>>>,
        weights: Option<Vec<Option<f64>>>,
        time: f64,
        shape_updates: Option<Vec<ShapeUpdate>>
    ) -> State {
        
        let xopt = self.xopt.clone();
        let xopt_core = self.xopt_core.clone();
        let mut rng: ThreadRng = thread_rng();

        // Update Goals for objectives if provided
        match goals {
            Some(new_goal_set) => {
                for i in 0..new_goal_set.len() {
                    match &new_goal_set[i] {
                        Some(goal) => self.objective_set.objectives[i].set_goal(goal),
                        _ => {}
                    }
                }
            },
            None => {}
        }

        // Update Weights for objectives if provided
        match weights {
            Some(new_weight_set) => {
                for i in 0..new_weight_set.len() {
                    match new_weight_set[i] {
                        Some(new_weight) => {
                            self.objective_set.objectives[i].set_weight(new_weight)
                        },
                        None => {}
                    }
                }
            },
            None => {}
        }

        // Update liveliness objectives based on time
        for objective in self.objective_set.objectives.iter_mut() {
            objective.update(time)
        }

        // Update the collision objects if provided
        match shape_updates {
            Some(updates) => self.robot_model.collision_manager.lock().unwrap().perform_updates(&updates),
            None => {}
        }

        // First, do xopt_core to develop a non-lively baseline
        self.xopt_core = self.solve_with_retries(xopt_core,true,self.only_core,&mut rng);
        self.vars.state_core = self.robot_model.get_state(&self.xopt_core,self.only_core);
        self.vars.history_core.update(&self.vars.state_core);


        if self.only_core {
            self.vars.history.update(&self.vars.state_core);
            //self.robot_model.collision_manager.lock().unwrap().update_ground_truth_table(&mut self.vars.state_core);
            return self.vars.state_core.clone()
        } else {
            self.xopt = self.solve_with_retries(xopt,false,true,&mut rng);
            let mut state = self.robot_model.get_state(&self.xopt,true);
            self.vars.history.update(&state);
            //self.robot_model.collision_manager.lock().unwrap().update_ground_truth_table(&mut state);
            
            
            return state
        }
    }
    
    pub fn solve_with_retries(
        &mut self,
        x: Vec<f64>,
        is_core: bool,
        is_last: bool,
        rng: &mut ThreadRng
    ) -> Vec<f64> {
        
        // Run until the max_retries is met, or the solution could be improved
        let mut best_cost = f64::INFINITY;
        let mut best_x = x.clone();
        let mut xopt = x.clone();
        let mut try_count = 0;

        while try_count < self.max_retries {

            if try_count > 0 {
                let mut i:usize = 0;
                for xval in xopt.iter_mut() {
                    if self.upper_bounds[i] - self.lower_bounds[i] > 0.0 {
                        *xval = 0.9*best_x[i]+0.1*rng.gen_range(self.lower_bounds[i]..self.upper_bounds[i]);
                        i+=1;
                    }
                }
            }

            let try_cost = optimize(
                &mut xopt, 
                &self.objective_set, 
                &self.robot_model, 
                &self.vars, 
                &mut self.panoc_cache, 
                &self.lower_bounds, 
                &self.upper_bounds, 
                self.max_iterations, 
                is_core,
                is_last
            );
            if try_cost < best_cost {
                best_x = xopt.clone();
                best_cost = try_cost;
            }
            try_count += 1;
        }
        return best_x.to_vec();
    }

    

    // fn call(&self, x: &[f64], is_core: bool) -> f64 {
    //     let mut out = 0.0;
    //     let state = self.robot_model.get_state(&x.to_vec());
    //     for i in 0..self.objective_set.objectives.len() {
    //         out += self.objective_set.objectives[i].call(&self.vars, &state, is_core);
    //     }
    //     out
    // }

    // fn gradient(
    //     &self,
    //     x: &[f64],
    //     is_core: bool,
    // ) -> (f64, Vec<f64>) {
    //     let mut grad: Vec<f64> = vec![0.; x.len()];
    //     let f_0 = self.call(x, is_core);

    //     for i in 0..x.len() {
    //         let mut x_h = x.to_vec();
    //         x_h[i] += 0.000001;
    //         let f_h = self.call(x_h.as_slice(), is_core);
    //         grad[i] = (-f_0 + f_h) / 0.000001;
    //     }

    //     (f_0, grad)
    // }

}

pub fn optimize(
    x: &mut [f64],
    objective_set: &ObjectiveSet,
    robot_model: &RobotModel,
    vars: &Vars,
    cache: &mut PANOCCache,
    lower_bounds: &Vec<f64>,
    upper_bounds: &Vec<f64>,
    max_iter: usize,
    is_core: bool,
    is_last: bool
) -> f64 {
    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        let (_my_obj, my_grad) = objective_set.gradient(&robot_model, &vars, u, is_core, is_last);
        for i in 0..my_grad.len() {
            grad[i] = my_grad[i];
        }
        Ok(())
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        *c = objective_set.call(&robot_model, &vars, u, is_core, is_last);
        Ok(())
    };

    let panoc_bounds = Rectangle::new(
        Option::from(lower_bounds.as_slice()),
        Option::from(upper_bounds.as_slice()),
    );

    /* PROBLEM STATEMENT */
    let problem = Problem::new(&panoc_bounds, df, f);
    let mut panoc = PANOCOptimizer::new(problem, cache)
        .with_max_iter(max_iter)
        .with_tolerance(0.0005);

    // Invoke the solver
    let result = panoc.solve(x);

    match result {
        Err(_err) => return f64::INFINITY.clone(),
        _ => return result.unwrap().cost_value(),
    }
    // println!("Status: {:?}",status);
}