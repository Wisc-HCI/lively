

# Tutorials

_NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!_


```jsx live

function StateGetterExample(props) {
    const [livelySolver, setLivelySolver] = useState(null);
    const [robotState, setRobotState] = useState(null)
    
    useEffect(()=>{

        const init = async ()=>{
            await lively.init();
            const newSolver = new lively.Solver(urdfs.ur3e,{});
            setLivelySolver(newSolver)
            setRobotState(newSolver.currentState)
        }
        init();
        
        return ()=>{}
    },[])

  return (
    <div>
       <Tree label='state' data={robotState}/>
    </div>
  );
}
```


