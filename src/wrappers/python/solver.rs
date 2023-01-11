// #[cfg(feature = "pybindings")]
// use pyo3::prelude::*;
// #[cfg(feature = "pybindings")]
// use crate::lively::Solver;
// #[cfg(feature = "pybindings")]
// use crate::objectives::objective::Objective;
// #[cfg(feature = "pybindings")]
// use crate::utils::shapes::*;
// #[cfg(feature = "pybindings")]
// use crate::utils::info::*;
// #[cfg(feature = "pybindings")]
// use crate::utils::state::State;
// #[cfg(feature = "pybindings")]
// use crate::utils::goals::Goal;
// #[cfg(feature = "pybindings")]
// use crate::wrappers::python::objectives::*;
// #[cfg(feature = "pybindings")]
// use crate::wrappers::python::shapes::*;
// #[cfg(feature = "pybindings")]
// use crate::wrappers::python::goals::*;
// #[cfg(feature = "pybindings")]
// use crate::wrappers::python::info::*;
// #[cfg(feature = "pybindings")]
// use crate::wrappers::python::geometry::*;
// #[cfg(feature = "pybindings")]
// use crate::wrappers::python::state::*;
// #[cfg(feature = "pybindings")]
// use std::collections::HashMap;

// #[cfg(feature = "pybindings")]
// #[pymethods]
// impl Solver {
//     #[new]
//     fn new(
//         urdf: String, 
//         objectives: std::collections::HashMap<String,PyObjective>, 
//         root_bounds: Option<Vec<PyScalarRange>>,
//         shapes: Option<Vec<PyShape>>,
//         initial_state: Option<PyState>,
//         max_retries: Option<usize>,
//         max_iterations: Option<usize>,
//         collision_settings: Option<PyCollisionSettingInfo>
//     ) -> Self {
//             //let inner_objectives =  std::collections::HashMap::from_iter(objectives.iter().map(|(s,o)| Objective::from(o.clone())).collect::<std::iter::Map<String,Objective>>());
//             let mut inner_objectives = HashMap::new();
//             for (key,value) in objectives{
//                 inner_objectives.insert(key,Objective::from(value.clone()));
//             }
//             let inner_shapes = shapes.map(|cs| cs.iter().map(|s| Shape::from(s.clone())).collect());
//             let inner_bounds = root_bounds.map(|bs| bs.iter().map(|b| (b.value,b.delta)).collect());
//             let inner_state = initial_state.map(|s| State::from(s));
//             let inner_collision_settings = collision_settings.map(|cs| CollisionSettingInfo::from(cs));
//             Solver::new(urdf, inner_objectives, inner_bounds, inner_shapes, inner_state, max_retries, max_iterations, inner_collision_settings)
//     }

//     #[getter(objectives)]
//     pub fn get_objectives_python(&self) -> PyResult<HashMap<String,PyObjective>>{
//         let mut objectives_hashmap = HashMap::new();
//             for (key,value) in &self.objective_set.objectives{
//                 objectives_hashmap.insert(key.to_string(),PyObjective::from(value.clone()));
//             }
//         return Ok(objectives_hashmap);
//     }

//     #[setter(objectives)]
//     pub fn set_objectives_python(&mut self, objectives: HashMap<String,PyObjective>) {
//         let mut objectives_hashmap = HashMap::new();
//             for (key,value) in objectives{
//                 objectives_hashmap.insert(key,Objective::from(value.clone()));
//             }
//         self.0.set_objectives(objectives_hashmap);
//     }

//     #[getter(current_state)]
//     pub fn get_current_state_python(&self) -> PyResult<State> {
//         Ok(PyState::from(self.0.get_current_state()))
//     }

//     #[getter(current_state)]
//     pub fn get_current_goals_python(&self) -> PyResult<HashMap<String,Option<PyGoal>>> {
//         let mut result_hashmap = HashMap::new();
//             for (key,value) in &self.0.objective_set.objectives{
//                 result_hashmap.insert(key.to_string(),Some(PyGoal::from(value.get_goal().unwrap().clone())));
//             }
//         return Ok(result_hashmap);

//     }

//     #[getter(links)]
//     pub fn get_links_python(&self) -> PyResult<Vec<PyLinkInfo>> {
//         Ok(self.0.robot_model.links.iter().map(|l| PyLinkInfo::from(l.clone())).collect())
//     }

//     #[getter(joints)]
//     pub fn get_joints_python(&self) -> PyResult<Vec<PyJointInfo>> {
//         Ok(self.0.robot_model.joints.iter().map(|j| PyJointInfo::from(j.clone())).collect())
//     }

//     fn reset_python(
//         &mut self, 
//         state: PyState,
//         weights: HashMap<String,f64>
//     ) -> PyResult<()> {

//         Ok(self.0.reset(State::from(state),weights))
//     }

//     fn solve_python(
//         &mut self,
//         goals:  HashMap<String,PyGoal>,
//         weights: HashMap<String,f64>,
//         time: f64,
//         shape_updates: Option<Vec<PyShapeUpdate>>
//     ) -> PyResult<PyState> {
//         let mut inner_goals: std::collections::HashMap<String, Goal> = HashMap::new();
//         for (key,goal) in goals{
//             inner_goals.insert(key,Goal::from(goal));
//         }
//         // let inner_goals = goals.map(|gs| gs.iter().map(|og| og.as_ref().map(|g| Goal::from(g.clone()))).collect());
//          let inner_updates = shape_updates.map(|cs| cs.iter().map(|s| ShapeUpdate::from(s.clone())).collect());
//         Ok(PyState::from(self.0.solve(
//             inner_goals,
//             weights,
//             time,
//             inner_updates)))
        
//     }

//     fn compute_average_distance_table_python(&mut self) -> PyResult<Vec<PyProximityInfo>> {
//         Ok(self.0.compute_average_distance_table().iter().map(|p| PyProximityInfo::from(p.clone())).collect())
//     }
// }