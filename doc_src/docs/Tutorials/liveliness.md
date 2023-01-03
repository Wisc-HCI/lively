

# Adding Liveliness
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

_NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!_


<Tabs>
  <TabItem value="jsx" label="Live">

  ```jsx live
function InitializationExample(props) {
    const [livelySolver, setLivelySolver] = useState(null);
    const [robot, setRobot] = useState('panda');
    const [robotState, setRobotState] = useState(null)
    
    useEffect(()=>{
        /* 
        Given that we are showing this example in a declarative
        react context, we need to use the useEffect hook to execute
        imperative (sequential) code. That means that if you are
        writing standard javascript, your code will look like the
        contents of the "init" function.
        * Note also that the "init" function is async. This is
        because the lively library is built on web assembly (WASM),
        which needs to be imported asynchronously.
        */
        const init = async ()=>{
            // Initialize the lively package (WASM)
            await lively.init();
            // Instantiate a new solver
            const newSolver = new lively.Solver(
                urdfs[robot], // The urdf of the robot
                {
                    'smoothness': {  // An example objective (smoothness macro)
                        name: 'MySmoothnessObjective',
                        type: 'SmoothnessMacro',
                        weight: 5
                    }
                }
            );
            // Assign the solver to the value
            setLivelySolver(newSolver)
            // Run solve to get a solved state
            const newState = newSolver.solve({},{},0.0);
            // Update the solver's current state
            setRobotState(newState)
        }
        init();
        
        return ()=>{
            // Provide a function to clear previous values
            setLivelySolver(null);
            setRobotState(null);
        }
    },[robot]) // Rerun this code if the robot changes

  return (
    <div>
       <Button active={robot==='panda'} onClick={()=>setRobot('panda')}>Panda</Button>
       <Button active={robot==='ur3e'} onClick={()=>setRobot('ur3e')}>UR3e</Button>
       <RobotViewer state={robotState} links={livelySolver ? livelySolver.links : []}/>
       <Tree label='state' data={robotState}/>
    </div>
  );
}
  ```

  </TabItem>

  <TabItem value="js" label="Javascript">

  ```js
import init, {Solver} from 'lively';

async function start() {
    // Initialize the lively package (WASM)
    await init();
    // Instantiate a new solver
    let solver = new Solver(
        "<?xml version='1.0' ?><robot name='panda'>...</robot>", // Full urdf as a string
        {
            'smoothness': {  // An example objective (smoothness macro)
                name: 'MySmoothnessObjective',
                type: 'SmoothnessMacro',
                weight: 5
            }
        }
    );
    // Run solve to get a solved state
    let state = solver.solve({},{},0.0);
    // Log the initial state
    console.log(state)
}

// Could be executed from anywhere that supports async actions
start();

  ```

  </TabItem>

  <TabItem value="py" label="Python">

  ```py
from lively import Solver, SmoothnessMacroObjective

# Instantiate a new solver
solver = Solver(
    urdf='<?xml version="1.0" ?><robot name="panda">...</robot>', # Full urdf as a string
    objectives={
        # An example objective (smoothness macro)
        "smoothness":SmoothnessMacroObjective(name="MySmoothnessObjective",weight=5)
    }
)

# Run solve to get a solved state
state = solver.solve({},{},0.0)
# Log the initial state
print(state)
  ```

  </TabItem>

  <TabItem value="rs" label="Rust">

  ```rust
use lively::lively::Solver;
use lively::objectives::core::base::SmoothnessMacroObjective;
use lively::objectives::objective::Objective;
use std::collections::HashMap;

// Create a map of objectives
let mut objectives: HashMap<String,Objective> = HashMap::new();
// Add a Smoothness Macro Objective
objectives.insert(
    "smoothness".into(),
    // An example objective (smoothness macro)
    Objective::SmoothnessMacro(SmoothnessMacroObjective::new("MySmoothnessObjective",5.0))
);

// Instantiate a new solver struct
let mut solver = Solver::new(
    urdf:'<?xml version="1.0" ?><robot name="panda">...</robot>', // Full urdf as a string
    objectives
);

// Run solve to get a solved state
let state = solver.solve(
    HashMap::new(), 
    HashMap::new(), 
    0.0, 
    None
);
// Log the initial state
println!("{:?}",state);
  ```

  </TabItem>

</Tabs> 





