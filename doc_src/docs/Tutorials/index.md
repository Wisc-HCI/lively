

# Tutorials

_NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!_


```jsx live

function StateGetterExample(props) {
    const [livelySolver, setLivelySolver] = useState(null);
    const [robot, setRobot] = useState('ur3e');
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
            // Instantiate a new solver struct
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
            // Update the solver's current state
            setRobotState(newSolver.currentState)
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


