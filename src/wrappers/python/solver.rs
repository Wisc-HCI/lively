use pyo3::prelude::*;
use crate::lively_tk::Solver;
use crate::objectives::objective::Objective;
use crate::utils::shapes::*;
use crate::utils::state::State;
use crate::utils::goals::Goal;
use crate::wrappers::python::objectives::*;
use crate::wrappers::python::shapes::*;
use crate::wrappers::python::goals::*;
use crate::wrappers::python::state::*;

#[pyclass(name="Solver")] 
pub struct PySolver(Solver);

#[pymethods]
impl PySolver {
    #[new]
    fn new(
        urdf: String, 
        objectives: Vec<PyObjective>, 
        root_bounds: Option<Vec<[f64; 2]>>,
        collision_shapes: Option<Vec<PyShape>>,
        zones: Option<Vec<PyZone>>,
        initial_state: Option<PyState>
    ) -> Self {
            let inner_objectives = objectives.iter().map(|o| Objective::from(o.clone())).collect();
            let inner_shapes = collision_shapes.map(|cs| cs.iter().map(|s| Shape::from(s.clone())).collect());
            let inner_zones = zones.map(|zs| zs.iter().map(|z| Zone::from(z.clone())).collect());
            let inner_state = initial_state.map(|s| State::from(s));
            PySolver(Solver::new(urdf, inner_objectives, root_bounds, inner_shapes, inner_zones, inner_state))
    }

    #[getter]
    pub fn get_objectives(&self) -> PyResult<Vec<PyObjective>>{
        Ok(self.0.objective_set.objectives.iter().map(|o| PyObjective::from(o.clone())).collect())
    }

    fn reset(
        &mut self, 
        state: PyState,
        weights: Option<Vec<Option<f64>>>
    ) -> PyResult<()> {
        Ok(self.0.reset(State::from(state),weights))
    }

    fn solve(
        &mut self,
        goals: Option<Vec<Option<PyGoal>>>,
        weights: Option<Vec<Option<f64>>>,
        time: f64,
        collision_shapes: Option<Vec<PyShape>>,
        zones: Option<Vec<PyZone>>,
        max_retries: Option<u64>,
        max_iterations: Option<usize>,
        only_core: Option<bool>
    ) -> PyResult<PyState> {
        let inner_goals = goals.map(|gs| gs.iter().map(|og| og.as_ref().map(|g| Goal::from(g.clone()))).collect());
        let inner_shapes = collision_shapes.map(|cs| cs.iter().map(|s| Shape::from(s.clone())).collect());
        let inner_zones = zones.map(|zs| zs.iter().map(|z| Zone::from(z.clone())).collect());
        Ok(PyState::from(self.0.solve(
            inner_goals,
            weights,
            time,
            inner_shapes,
            inner_zones,
            max_retries,max_iterations,only_core)))
    }
}