# Properties


| Property | Description |
| --- | --- | 
|`robot_model`|A set of information and properties of a robot|
|`vars`|A set of useful features within the optimization, including history, joint, and link information|
|`lower_bounds`|A vector representing the lower bounds in the optimization vector. This is derived from the robot base limits and joint limits|
|`upper_bounds`|A vector representing the upper bounds in the optimization vector. This is derived from the robot base limits, as well as joint limits|
|`objective_set`|A lookup table containing all of the current objectives in use|
|`max_retries`|The maximum number of randomly initialized rounds allowed for each invocation of solve|
|`max_iterations`|The number of maximum iterations per round within the optimization|
|`get_current_state()`| Returns the solution [state](../../state) from the last invocation of `solve`|
|`get_objectives()`| Returns the current [objectives](../../Objectives/objective) as a lookup table|
|`set_objectives(new_objectives)`| Replace the current [objectives](../../Objectives/objective) with new objectives lookup table with `new_objectives` parameter |
|`get_goals()`| Returns the current [goals](../../Goals/goal) as a lookup table|
|`set_goals(new_goals)`| Replacce the current [goals](../../Goals/goal) with a lookup table of goals|



