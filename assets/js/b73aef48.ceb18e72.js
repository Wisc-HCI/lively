"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[2157],{3905:(e,t,n)=>{n.d(t,{Zo:()=>c,kt:()=>m});var a=n(67294);function i(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function l(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){i(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function r(e,t){if(null==e)return{};var n,a,i=function(e,t){if(null==e)return{};var n,a,i={},o=Object.keys(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||(i[n]=e[n]);return i}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(i[n]=e[n])}return i}var s=a.createContext({}),u=function(e){var t=a.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):l(l({},t),e)),n},c=function(e){var t=u(e.components);return a.createElement(s.Provider,{value:t},e.children)},v="mdxType",p={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},d=a.forwardRef((function(e,t){var n=e.components,i=e.mdxType,o=e.originalType,s=e.parentName,c=r(e,["components","mdxType","originalType","parentName"]),v=u(n),d=i,m=v["".concat(s,".").concat(d)]||v[d]||p[d]||o;return n?a.createElement(m,l(l({ref:t},c),{},{components:n})):a.createElement(m,l({ref:t},c))}));function m(e,t){var n=arguments,i=t&&t.mdxType;if("string"==typeof e||i){var o=n.length,l=new Array(o);l[0]=d;var r={};for(var s in t)hasOwnProperty.call(t,s)&&(r[s]=t[s]);r.originalType=e,r[v]="string"==typeof e?e:i,l[1]=r;for(var u=2;u<o;u++)l[u]=n[u];return a.createElement.apply(null,l)}return a.createElement.apply(null,n)}d.displayName="MDXCreateElement"},85162:(e,t,n)=>{n.d(t,{Z:()=>l});var a=n(67294),i=n(86010);const o="tabItem_Ymn6";function l(e){let{children:t,hidden:n,className:l}=e;return a.createElement("div",{role:"tabpanel",className:(0,i.Z)(o,l),hidden:n},t)}},74866:(e,t,n)=>{n.d(t,{Z:()=>w});var a=n(83117),i=n(67294),o=n(86010),l=n(12466),r=n(16550),s=n(91980),u=n(67392),c=n(50012);function v(e){return function(e){return i.Children.map(e,(e=>{if((0,i.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))}(e).map((e=>{let{props:{value:t,label:n,attributes:a,default:i}}=e;return{value:t,label:n,attributes:a,default:i}}))}function p(e){const{values:t,children:n}=e;return(0,i.useMemo)((()=>{const e=t??v(n);return function(e){const t=(0,u.l)(e,((e,t)=>e.value===t.value));if(t.length>0)throw new Error(`Docusaurus error: Duplicate values "${t.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[t,n])}function d(e){let{value:t,tabValues:n}=e;return n.some((e=>e.value===t))}function m(e){let{queryString:t=!1,groupId:n}=e;const a=(0,r.k6)(),o=function(e){let{queryString:t=!1,groupId:n}=e;if("string"==typeof t)return t;if(!1===t)return null;if(!0===t&&!n)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return n??null}({queryString:t,groupId:n});return[(0,s._X)(o),(0,i.useCallback)((e=>{if(!o)return;const t=new URLSearchParams(a.location.search);t.set(o,e),a.replace({...a.location,search:t.toString()})}),[o,a])]}function b(e){const{defaultValue:t,queryString:n=!1,groupId:a}=e,o=p(e),[l,r]=(0,i.useState)((()=>function(e){let{defaultValue:t,tabValues:n}=e;if(0===n.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(t){if(!d({value:t,tabValues:n}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${t}" but none of its children has the corresponding value. Available values are: ${n.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return t}const a=n.find((e=>e.default))??n[0];if(!a)throw new Error("Unexpected error: 0 tabValues");return a.value}({defaultValue:t,tabValues:o}))),[s,u]=m({queryString:n,groupId:a}),[v,b]=function(e){let{groupId:t}=e;const n=function(e){return e?`docusaurus.tab.${e}`:null}(t),[a,o]=(0,c.Nk)(n);return[a,(0,i.useCallback)((e=>{n&&o.set(e)}),[n,o])]}({groupId:a}),h=(()=>{const e=s??v;return d({value:e,tabValues:o})?e:null})();(0,i.useLayoutEffect)((()=>{h&&r(h)}),[h]);return{selectedValue:l,selectValue:(0,i.useCallback)((e=>{if(!d({value:e,tabValues:o}))throw new Error(`Can't select invalid tab value=${e}`);r(e),u(e),b(e)}),[u,b,o]),tabValues:o}}var h=n(72389);const y="tabList__CuJ",f="tabItem_LNqP";function g(e){let{className:t,block:n,selectedValue:r,selectValue:s,tabValues:u}=e;const c=[],{blockElementScrollPositionUntilNextRender:v}=(0,l.o5)(),p=e=>{const t=e.currentTarget,n=c.indexOf(t),a=u[n].value;a!==r&&(v(t),s(a))},d=e=>{let t=null;switch(e.key){case"Enter":p(e);break;case"ArrowRight":{const n=c.indexOf(e.currentTarget)+1;t=c[n]??c[0];break}case"ArrowLeft":{const n=c.indexOf(e.currentTarget)-1;t=c[n]??c[c.length-1];break}}t?.focus()};return i.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.Z)("tabs",{"tabs--block":n},t)},u.map((e=>{let{value:t,label:n,attributes:l}=e;return i.createElement("li",(0,a.Z)({role:"tab",tabIndex:r===t?0:-1,"aria-selected":r===t,key:t,ref:e=>c.push(e),onKeyDown:d,onClick:p},l,{className:(0,o.Z)("tabs__item",f,l?.className,{"tabs__item--active":r===t})}),n??t)})))}function k(e){let{lazy:t,children:n,selectedValue:a}=e;if(n=Array.isArray(n)?n:[n],t){const e=n.find((e=>e.props.value===a));return e?(0,i.cloneElement)(e,{className:"margin-top--md"}):null}return i.createElement("div",{className:"margin-top--md"},n.map(((e,t)=>(0,i.cloneElement)(e,{key:t,hidden:e.props.value!==a}))))}function S(e){const t=b(e);return i.createElement("div",{className:(0,o.Z)("tabs-container",y)},i.createElement(g,(0,a.Z)({},e,t)),i.createElement(k,(0,a.Z)({},e,t)))}function w(e){const t=(0,h.Z)();return i.createElement(S,(0,a.Z)({key:String(t)},e))}},74732:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>c,contentTitle:()=>s,default:()=>d,frontMatter:()=>r,metadata:()=>u,toc:()=>v});var a=n(83117),i=(n(67294),n(3905)),o=n(74866),l=n(85162);const r={},s="Adding Liveliness",u={unversionedId:"Tutorials/liveliness",id:"Tutorials/liveliness",title:"Adding Liveliness",description:"Since Lively is still in beta, the design is subject to change and should not be considered final!",source:"@site/docs/Tutorials/liveliness.mdx",sourceDirName:"Tutorials",slug:"/Tutorials/liveliness",permalink:"/lively/docs/Tutorials/liveliness",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"tutorials",previous:{title:"Solving",permalink:"/lively/docs/Tutorials/solving"},next:{title:"Social Robot",permalink:"/lively/docs/Tutorials/social"}},c={},v=[],p={toc:v};function d(e){let{components:t,...n}=e;return(0,i.kt)("wrapper",(0,a.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,i.kt)("h1",{id:"adding-liveliness"},"Adding Liveliness"),(0,i.kt)("admonition",{type:"note"},(0,i.kt)("p",{parentName:"admonition"},"Since Lively is still in beta, the design is subject to change and should not be considered final!")),(0,i.kt)("p",null,"We have also created examples in Javascript, Python, and Rust for liveliness. You can find the file by clicking the links in the table down below."),(0,i.kt)("table",null,(0,i.kt)("thead",{parentName:"table"},(0,i.kt)("tr",{parentName:"thead"},(0,i.kt)("th",{parentName:"tr",align:null},"Language"),(0,i.kt)("th",{parentName:"tr",align:null},"Path"),(0,i.kt)("th",{parentName:"tr",align:null},"Command to run the example"))),(0,i.kt)("tbody",{parentName:"table"},(0,i.kt)("tr",{parentName:"tbody"},(0,i.kt)("td",{parentName:"tr",align:null},"Rust"),(0,i.kt)("td",{parentName:"tr",align:null},(0,i.kt)("a",{parentName:"td",href:"https://github.com/Wisc-HCI/lively/blob/master/examples/rust_examples/liveliness_example.rs"},"link")),(0,i.kt)("td",{parentName:"tr",align:null},"cargo run --package lively --example liveliness_example")),(0,i.kt)("tr",{parentName:"tbody"},(0,i.kt)("td",{parentName:"tr",align:null},"Python"),(0,i.kt)("td",{parentName:"tr",align:null},(0,i.kt)("a",{parentName:"td",href:"https://github.com/Wisc-HCI/lively/blob/master/examples/python_examples/liveliness_example.ipynb"},"link")),(0,i.kt)("td",{parentName:"tr",align:null},"run in the Jupyter Notebook")),(0,i.kt)("tr",{parentName:"tbody"},(0,i.kt)("td",{parentName:"tr",align:null},"Javascript"),(0,i.kt)("td",{parentName:"tr",align:null},(0,i.kt)("a",{parentName:"td",href:"https://github.com/Wisc-HCI/lively/tree/master/examples/js_examples/liveliness"},"link")),(0,i.kt)("td",{parentName:"tr",align:null},(0,i.kt)("inlineCode",{parentName:"td"},"yarn build"),", ",(0,i.kt)("inlineCode",{parentName:"td"},"yarn dev"))))),(0,i.kt)(o.Z,{mdxType:"Tabs"},(0,i.kt)(l.Z,{value:"jsx",label:"Live",mdxType:"TabItem"},(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-jsx",metastring:"live",live:!0},'function InitializationExample(props) {\n  const [livelySolver, setLivelySolver] = useState(null);\n  const [robot, setRobot] = useState("panda");\n  const [robotState, setRobotState] = useState(null);\n  const [visualRobotState, setVisualRobotState] = useState(null);\n\n  useEffect(() => {\n    /* \n      Given that we are showing this example in a declarative\n      react context, we need to use the useEffect hook to execute\n      imperative (sequential) code. That means that if you are\n      writing standard javascript, your code will look like the\n      contents of the function in this \'useEffect\'.\n    */\n\n    const newSolver = new lively.Solver(\n      urdfs.panda, // The urdf of the robot\n      {\n        // some lively objective examples. Notice for JavaScript, you do not need to import anything for objective. Simply construct an object\n        smoothness: {\n          name: "MySmoothnessObjective",\n          type: "SmoothnessMacro",\n          weight: 20,\n          joints: true,\n          origin: false,\n          links: true,\n        },\n        collision: {\n          name: "MyCollisionDetection",\n          type: "CollisionAvoidance",\n          weight: 5,\n        },\n        jointLimit: {\n          name: "MyJointLimit",\n          type: "JointLimits",\n          weight: 5,\n        },\n        //collision, jointlimit\n        positionLively: {\n          name: "MyLivelinessObjective",\n          type: "PositionLiveliness",\n          weight: 15,\n          link: "panda_hand",\n          frequency: 7,\n        },\n        position: {\n          name: "MyPositionObjective",\n          type: "PositionMatch",\n          weight: 10,\n          link: \'panda_hand\'\n        },\n        orientationLively: {\n          name: "MyOrientationObjective",\n          type: "OrientationLiveliness",\n          weight: 15,\n          link: "panda_link2",\n          frequency: 7,\n        },\n        finger_joint_control: {\n          name: "FingerJointControl",\n          type: "JointMatch",\n          weight: 25,\n          joint: "panda_finger_joint1",\n        },\n      }\n    );\n    // Normalize the collision bodies\n    newSolver.computeAverageDistanceTable();\n    // Assign the solver to the value\n    setLivelySolver(newSolver);\n    // Run solve to get a solved state\n    const newState = newSolver.solve({}, {}, 0.0);\n    // Update the solver\'s current state\n    setRobotState(newState);\n    setVisualRobotState(newState);\n\n    const updateSolver = () => {\n      if (newSolver) {\n        const d = new Date();\n        let time = d.getTime(); // Get the time in milliseconds\n        let goals = {\n          // An goal example with defined Scalar and Size values for the lively objectives\n          position: {\n            Translation: [0.6,0,0.6]\n          },\n          finger_joint_control: {\n            Scalar: 0.02,\n          },\n          positionLively: {\n            Size: [0.07, 0.05, 0.08],\n          },\n          orientationLively: {\n            Size: [0.07, 0.05, 0.08],\n          },\n          jointLimit: {\n            Scalar: 0.02,\n          },\n        };\n        const newState = newSolver.solve(goals, {}, time / 1000); // Convert the time to second\n        // Update the solver\'s current state\n\n        setRobotState(newState);\n      }\n    };\n\n    const interval = setInterval(updateSolver, 1000 / 30); // Update the solver 30fps\n\n    return () => {\n      // Provide a function to clear previous values\n      setLivelySolver(null);\n      setRobotState(null);\n      setVisualRobotState(null);\n      clearInterval(interval);\n    };\n  }, [robot]); // Rerun this code if the robot changes\n\n  return (\n    <div>\n      <RobotViewer\n        state={robotState}\n        links={livelySolver ? livelySolver.links : []}\n      />\n      <Button\n        active={robot === "panda"}\n        onClick={() => setVisualRobotState(robotState)}\n      >\n        Update State\n      </Button>\n      <Tree label="state" data={visualRobotState} />\n    </div>\n  );\n}\n'))),(0,i.kt)(l.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-js"},'import init, { Solver } from "lively";\n\nasync function start() {\n  // Initialize the lively package (WASM)\n  await init();\n  // Instantiate a new solver\n  let solver = new Solver(\n    "<?xml version=\'1.0\' ?><robot name=\'panda\'>...</robot>", // Full urdf as a string\n    {\n      smoothness: {\n        // An example objective (smoothness macro)\n        name: "MySmoothnessObjective",\n        type: "SmoothnessMacro",\n        weight: 5,\n      },\n      position: {\n        name: "MyPositionObjective",\n        type: "PositionMatch",\n        weight: 15,\n        link: \'panda_hand\'\n      },\n      positionLively: {\n        name: "MyLivelinessObjective",\n        type: "PositionLiveliness",\n        weight: 15,\n        link: "panda_hand",\n        frequency: 7,\n      },\n      orientationLively: {\n        name: "MyOrientationObjective",\n        type: "OrientationLiveliness",\n        weight: 15,\n        link: "panda_link2",\n        frequency: 7,\n      },\n    }\n  );\n  // Run solve to get a solved state\n  let state = solver.solve({}, {}, 0.0);\n  // Log the initial state\n  console.log(state);\n}\n\n// Could be executed from anywhere that supports async actions\nstart();\n'))),(0,i.kt)(l.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-py"},'from lively import Solver, SmoothnessMacroObjective\n\n# Instantiate a new solver\nsolver = Solver(\n  urdf=\'<?xml version="1.0" ?><robot name="panda">...</robot>\', # Full urdf as a string\n  objectives={\n      # An example smoothness macro objective\n      "smoothness":SmoothnessMacroObjective(name="MySmoothnessObjective",weight=5),\n      # An example position liveliness objective\n      "positionLiveliness": PositionLiveliness(name="MyLivelinessObjective",link="panda_hand",frequency=7,weight=25),\n      # An example orientation liveliness objective\n      "orientationLiveliness": OrientationLiveliness(name="myOrientationLiveliness",link="panda_link2",frequency=7,weight=15)\n  }\n)\n\n# Run solve to get a solved state\nstate = solver.solve({},{},0.0)\n# Log the initial state\nprint(state)\n'))),(0,i.kt)(l.Z,{value:"rs",label:"Rust",mdxType:"TabItem"},(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-rust"},'use lively::lively::Solver;\nuse lively::objectives::core::base::SmoothnessMacroObjective;\nuse lively::objectives::objective::Objective;\nuse std::collections::HashMap;\n\n// Create a map of objectives\nlet mut objectives: HashMap<String,Objective> = HashMap::new();\n// Add a Smoothness Macro Objective\nobjectives.insert("smoothness".into(),Objective::SmoothnessMacro(SmoothnessMacroObjective::new("MySmoothnessObjective",5.0)));\n// Add a positionLiveliness Objective\nobjectives.insert(("positionLiveliness".into(),Objective::PositionLiveliness(PositionLiveliness::new("MyLivelinessObjective",25,"panda_hand",7))));\n// Add a orientationLiveliness Objective\nobjectives.insert(("orientationLiveliness".into(),Objective::OrientationLiveliness(OrientationLiveliness::new("MyOrientationlinessObjective",15,"panda_link2",7))));\n\n\n\n// Instantiate a new solver struct\nlet mut solver = Solver::new(\n  urdf:\'<?xml version="1.0" ?><robot name="panda">...</robot>\', // Full urdf as a string\n  objectives\n);\n\n// Run solve to get a solved state\nlet state = solver.solve(\n  HashMap::new(),\n  HashMap::new(),\n  0.0,\n  None\n);\n// Log the initial state\nprintln!("{:?}",state);\n')))))}d.isMDXComponent=!0}}]);