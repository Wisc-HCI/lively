import { Solver } from "../../../../pkg";
import { useState, useEffect } from "react";

export const useLively = ({
  urdf,
  objectives,
  rootBounds,
  shapes,
  initialState,
}) => {
  const [livelySolver, setLivelySolver] = useState(null);

  const solve = (goals, weights, time, shapeUpdates) => {
    if (livelySolver) {
      return livelySolver.solve(
        goals,
        weights,
        time,
        shapeUpdates // compare to static environmental shapes, these shapes can be modified.
      ); // Pass the new goal into solve function
    }
  };

  useEffect(() => {
    /* 
        Given that we are showing this example in a declarative
        react context, we need to use the useEffect hook to execute
        imperative (sequential) code. That means that if you are
        writing standard javascript, your code will look like the
        contents of the function in this 'useEffect'.
    */
    const newSolver = new Solver(
      urdf, // The urdf of the robot
      objectives,
      rootBounds,
      shapes, // all the shapes passed in here will be considered as the static environmental shapes. This means that these shapes can not be modified through shapes_update in solve.
      initialState
    );
    newSolver.computeAverageDistanceTable();
    // Assign the solver to the value
    setLivelySolver(newSolver);

    return () => {
      // Provide a function to clear previous values
      setLivelySolver(null);
    };
  }, [
    urdf,
    JSON.stringify(objectives),
    JSON.stringify(rootBounds),
    JSON.stringify(shapes),
    JSON.stringify(initialState),
  ]); // Rerun this code if the props change

  return {
    solve,
    solver: livelySolver,
  };
};
