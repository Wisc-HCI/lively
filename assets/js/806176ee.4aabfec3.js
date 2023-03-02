"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[567],{3905:(n,e,t)=>{t.d(e,{Zo:()=>c,kt:()=>h});var o=t(67294);function r(n,e,t){return e in n?Object.defineProperty(n,e,{value:t,enumerable:!0,configurable:!0,writable:!0}):n[e]=t,n}function a(n,e){var t=Object.keys(n);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(n);e&&(o=o.filter((function(e){return Object.getOwnPropertyDescriptor(n,e).enumerable}))),t.push.apply(t,o)}return t}function s(n){for(var e=1;e<arguments.length;e++){var t=null!=arguments[e]?arguments[e]:{};e%2?a(Object(t),!0).forEach((function(e){r(n,e,t[e])})):Object.getOwnPropertyDescriptors?Object.defineProperties(n,Object.getOwnPropertyDescriptors(t)):a(Object(t)).forEach((function(e){Object.defineProperty(n,e,Object.getOwnPropertyDescriptor(t,e))}))}return n}function i(n,e){if(null==n)return{};var t,o,r=function(n,e){if(null==n)return{};var t,o,r={},a=Object.keys(n);for(o=0;o<a.length;o++)t=a[o],e.indexOf(t)>=0||(r[t]=n[t]);return r}(n,e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(n);for(o=0;o<a.length;o++)t=a[o],e.indexOf(t)>=0||Object.prototype.propertyIsEnumerable.call(n,t)&&(r[t]=n[t])}return r}var l=o.createContext({}),u=function(n){var e=o.useContext(l),t=e;return n&&(t="function"==typeof n?n(e):s(s({},e),n)),t},c=function(n){var e=u(n.components);return o.createElement(l.Provider,{value:e},n.children)},m="mdxType",f={inlineCode:"code",wrapper:function(n){var e=n.children;return o.createElement(o.Fragment,{},e)}},p=o.forwardRef((function(n,e){var t=n.components,r=n.mdxType,a=n.originalType,l=n.parentName,c=i(n,["components","mdxType","originalType","parentName"]),m=u(t),p=r,h=m["".concat(l,".").concat(p)]||m[p]||f[p]||a;return t?o.createElement(h,s(s({ref:e},c),{},{components:t})):o.createElement(h,s({ref:e},c))}));function h(n,e){var t=arguments,r=e&&e.mdxType;if("string"==typeof n||r){var a=t.length,s=new Array(a);s[0]=p;var i={};for(var l in e)hasOwnProperty.call(e,l)&&(i[l]=e[l]);i.originalType=n,i[m]="string"==typeof n?n:r,s[1]=i;for(var u=2;u<a;u++)s[u]=t[u];return o.createElement.apply(null,s)}return o.createElement.apply(null,t)}p.displayName="MDXCreateElement"},85162:(n,e,t)=>{t.d(e,{Z:()=>s});var o=t(67294),r=t(86010);const a="tabItem_Ymn6";function s(n){let{children:e,hidden:t,className:s}=n;return o.createElement("div",{role:"tabpanel",className:(0,r.Z)(a,s),hidden:t},e)}},74866:(n,e,t)=>{t.d(e,{Z:()=>g});var o=t(83117),r=t(67294),a=t(86010),s=t(12466),i=t(16550),l=t(91980),u=t(67392),c=t(50012);function m(n){return function(n){return r.Children.map(n,(n=>{if((0,r.isValidElement)(n)&&"value"in n.props)return n;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof n.type?n.type:n.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))}(n).map((n=>{let{props:{value:e,label:t,attributes:o,default:r}}=n;return{value:e,label:t,attributes:o,default:r}}))}function f(n){const{values:e,children:t}=n;return(0,r.useMemo)((()=>{const n=e??m(t);return function(n){const e=(0,u.l)(n,((n,e)=>n.value===e.value));if(e.length>0)throw new Error(`Docusaurus error: Duplicate values "${e.map((n=>n.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(n),n}),[e,t])}function p(n){let{value:e,tabValues:t}=n;return t.some((n=>n.value===e))}function h(n){let{queryString:e=!1,groupId:t}=n;const o=(0,i.k6)(),a=function(n){let{queryString:e=!1,groupId:t}=n;if("string"==typeof e)return e;if(!1===e)return null;if(!0===e&&!t)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return t??null}({queryString:e,groupId:t});return[(0,l._X)(a),(0,r.useCallback)((n=>{if(!a)return;const e=new URLSearchParams(o.location.search);e.set(a,n),o.replace({...o.location,search:e.toString()})}),[a,o])]}function d(n){const{defaultValue:e,queryString:t=!1,groupId:o}=n,a=f(n),[s,i]=(0,r.useState)((()=>function(n){let{defaultValue:e,tabValues:t}=n;if(0===t.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(e){if(!p({value:e,tabValues:t}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${e}" but none of its children has the corresponding value. Available values are: ${t.map((n=>n.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return e}const o=t.find((n=>n.default))??t[0];if(!o)throw new Error("Unexpected error: 0 tabValues");return o.value}({defaultValue:e,tabValues:a}))),[l,u]=h({queryString:t,groupId:o}),[m,d]=function(n){let{groupId:e}=n;const t=function(n){return n?`docusaurus.tab.${n}`:null}(e),[o,a]=(0,c.Nk)(t);return[o,(0,r.useCallback)((n=>{t&&a.set(n)}),[t,a])]}({groupId:o}),v=(()=>{const n=l??m;return p({value:n,tabValues:a})?n:null})();(0,r.useLayoutEffect)((()=>{v&&i(v)}),[v]);return{selectedValue:s,selectValue:(0,r.useCallback)((n=>{if(!p({value:n,tabValues:a}))throw new Error(`Can't select invalid tab value=${n}`);i(n),u(n),d(n)}),[u,d,a]),tabValues:a}}var v=t(72389);const b="tabList__CuJ",w="tabItem_LNqP";function y(n){let{className:e,block:t,selectedValue:i,selectValue:l,tabValues:u}=n;const c=[],{blockElementScrollPositionUntilNextRender:m}=(0,s.o5)(),f=n=>{const e=n.currentTarget,t=c.indexOf(e),o=u[t].value;o!==i&&(m(e),l(o))},p=n=>{let e=null;switch(n.key){case"Enter":f(n);break;case"ArrowRight":{const t=c.indexOf(n.currentTarget)+1;e=c[t]??c[0];break}case"ArrowLeft":{const t=c.indexOf(n.currentTarget)-1;e=c[t]??c[c.length-1];break}}e?.focus()};return r.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,a.Z)("tabs",{"tabs--block":t},e)},u.map((n=>{let{value:e,label:t,attributes:s}=n;return r.createElement("li",(0,o.Z)({role:"tab",tabIndex:i===e?0:-1,"aria-selected":i===e,key:e,ref:n=>c.push(n),onKeyDown:p,onClick:f},s,{className:(0,a.Z)("tabs__item",w,s?.className,{"tabs__item--active":i===e})}),t??e)})))}function T(n){let{lazy:e,children:t,selectedValue:o}=n;if(t=Array.isArray(t)?t:[t],e){const n=t.find((n=>n.props.value===o));return n?(0,r.cloneElement)(n,{className:"margin-top--md"}):null}return r.createElement("div",{className:"margin-top--md"},t.map(((n,e)=>(0,r.cloneElement)(n,{key:e,hidden:n.props.value!==o}))))}function S(n){const e=d(n);return r.createElement("div",{className:(0,a.Z)("tabs-container",b)},r.createElement(y,(0,o.Z)({},n,e)),r.createElement(T,(0,o.Z)({},n,e)))}function g(n){const e=(0,v.Z)();return r.createElement(S,(0,o.Z)({key:String(e)},n))}},76496:(n,e,t)=>{t.r(e),t.d(e,{assets:()=>c,contentTitle:()=>l,default:()=>p,frontMatter:()=>i,metadata:()=>u,toc:()=>m});var o=t(83117),r=(t(67294),t(3905)),a=t(74866),s=t(85162);const i={},l="Environment Modeling-sine",u={unversionedId:"Tutorials/environment-sine",id:"Tutorials/environment-sine",title:"Environment Modeling-sine",description:"Since Lively is still in beta, the design is subject to change and should not be considered final!",source:"@site/docs/Tutorials/environment-sine.mdx",sourceDirName:"Tutorials",slug:"/Tutorials/environment-sine",permalink:"/lively/docs/Tutorials/environment-sine",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"tutorials",previous:{title:"Environment Modeling",permalink:"/lively/docs/Tutorials/environment"},next:{title:"Customized Usage",permalink:"/lively/docs/category/customized-usage"}},c={},m=[],f={toc:m};function p(n){let{components:e,...t}=n;return(0,r.kt)("wrapper",(0,o.Z)({},f,t,{components:e,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"environment-modeling-sine"},"Environment Modeling-sine"),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"Since Lively is still in beta, the design is subject to change and should not be considered final!")),(0,r.kt)(a.Z,{mdxType:"Tabs"},(0,r.kt)(s.Z,{value:"jsx",label:"Live",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-jsx",metastring:"live",live:!0},'function InitializationExample(props) {\n  const initialGoalValues = {\n    // The initial translation of the sphere is at where the gripper is supposed to be at.\n    translation: [\n      -0.35289461347986373, 0.13931635901132902, 0.38946876678037234,\n    ],\n    // The initial rotation is pointing to where the gripper is pointing at.\n    rotation: [\n      0.968947708849441, 0.01867797756065113, -0.2465018112869008,\n      0.005322377470646942,\n    ],\n  };\n\n  const initialState = {\n    origin: {\n      translation: [0.0, 0.0, 0.0],\n      rotation: [0.0, 0.0, 0.0, 1.0],\n    },\n    joints: {\n      shoulder_pan_joint: 3.054197606864953,\n      elbow_joint: -1.2683111703953787,\n      wrist_3_joint: -4.823194599105927,\n      robotiq_85_left_finger_tip_joint: -0.051922213806626316,\n      robotiq_85_right_inner_knuckle_joint: 0.051922213806626316,\n      robotiq_85_right_finger_tip_joint: -0.051922213806626316,\n      shoulder_lift_joint: -1.748886737780085,\n      wrist_1_joint: 5.08275093442812,\n      robotiq_85_left_inner_knuckle_joint: 0.051922213806626316,\n      robotiq_85_right_knuckle_joint: 0.051922213806626316,\n      robotiq_85_left_knuckle_joint: 0.051922213806626316,\n      wrist_2_joint: -4.651476380979644,\n    },\n  };\n\n  const arrowTransformControl = {\n    // the visual reresentation for the rotation transform control.\n    type: "Arrow",\n    name: "arrow transform control",\n    frame: "world",\n    physical: false,\n    localTransform: initialGoalValues,\n  };\n\n  const sphereTransformControl = {\n    // the visual reresentation for the translation transform control.\n    type: "Sphere",\n    name: "sphere transform control",\n    frame: "world",\n    radius: 0.05,\n    localTransform: initialGoalValues,\n  };\n\n  const initialGoal = {\n    // the inital goal to be fed into the the solver representing the the initial translation and rotation.\n    position: {\n      Translation: initialGoalValues.translation,\n    },\n    orientation: {\n      Rotation: initialGoalValues.rotation,\n    },\n  };\n\n  const initialEnvShapes = [];\n\n  const [livelySolver, setLivelySolver] = useState(null);\n  const [robot, setRobot] = useState("panda");\n  const [robotState, setRobotState] = useState(null);\n  const [visualRobotState, setVisualRobotState] = useState(null);\n  const [transformMode, setTransformMode] = useState("translate");\n  const [activeShapesTransform, setShapesTransform] = useState(false);\n  const [arrowTransformControlVisual, setArrowTransformControlVisual] =\n    useState(arrowTransformControl);\n  const [sphereTransformControlVisual, setSphereTransformControlVisual] =\n    useState(sphereTransformControl);\n  const [transformControl, setTransformControl] = useState(\n    sphereTransformControl\n  ); // the visual representation of the transformMode.\n  const [goal, setGoal] = useState(initialGoal);\n  const [envShapes, setEnvShapes] = useState(initialEnvShapes);\n  const [shapesUpdate, setShapesUpdate] = useState(null);\n  const [showCollision, setShowCollision] = useState(false);\n\n  function updateTransformMode() {\n    // switch between Position Match or Orientation Match\n    transformMode === "translate"\n      ? setTransformMode("rotate")\n      : setTransformMode("translate");\n    transformControl.type === "Arrow"\n      ? setTransformControl(sphereTransformControlVisual)\n      : setTransformControl(arrowTransformControlVisual);\n  }\n\n  function updateTransformControlTransform(transform) {\n    let newGoal = goal;\n    newGoal.position.Translation = [\n      transform.position.x,\n      transform.position.y,\n      transform.position.z,\n    ];\n    newGoal.orientation.Rotation = [\n      transform.quaternion.x,\n      transform.quaternion.y,\n      transform.quaternion.z,\n      transform.quaternion.w,\n    ];\n    let newArrowTransformControl = arrowTransformControlVisual;\n    let newSphereTransformControl = sphereTransformControlVisual;\n    newSphereTransformControl.localTransform.translation = [\n      transform.position.x,\n      transform.position.y,\n      transform.position.z,\n    ];\n    newSphereTransformControl.localTransform.rotation = [\n      transform.quaternion.x,\n      transform.quaternion.y,\n      transform.quaternion.z,\n      transform.quaternion.w,\n    ];\n    newArrowTransformControl.localTransform.translation = [\n      transform.position.x,\n      transform.position.y,\n      transform.position.z,\n    ];\n    newArrowTransformControl.localTransform.rotation = [\n      transform.quaternion.x,\n      transform.quaternion.y,\n      transform.quaternion.z,\n      transform.quaternion.w,\n    ];\n    setArrowTransformControlVisual(newArrowTransformControl); // update the the transformation of the arrow transform control visual in the scene.\n    setSphereTransformControlVisual(newSphereTransformControl); // update the the transformation of the sphere transform control visual in the scene.\n    setGoal(newGoal); // update the goal being sent to the solver.\n    transformMode === "translate"\n      ? setTransformControl(newSphereTransformControl)\n      : setTransformControl(newArrowTransformControl);\n  }\n\n  function updateTransform(id, transform) {\n    if (id.includes(`env-shape`)) {\n      updateEnvironmentalShapesTransform(id, transform);\n    } else {\n      updateTransformControlTransform(transform);\n    }\n  }\n\n  useEffect(() => {\n    /*\n      Given that we are showing this example in a declarative\n      react context, we need to use the useEffect hook to execute\n      imperative (sequential) code. That means that if you are\n      writing standard javascript, your code will look like the\n      contents of the "init" function.\n      * Note also that the "init" function is async. This is\n      because the lively library is built on web assembly (WASM),\n      which needs to be imported asynchronously.\n      */\n    var currentSolver;\n\n    // Instantiate a new solver\n    const newSolver = new lively.Solver(\n      urdfs.ur3e, // The urdf of the robot\n      {\n        // some objective examples. Notice for JavaScript, you do not need to import anything for objective. Simply construct an object\n        smoothness: {\n          name: "MySmoothnessObjective",\n          type: "SmoothnessMacro",\n          weight: 15,\n          joints: true,\n          origin: false,\n          links: true,\n        },\n        collision: {\n          // The main objective that allows the robot to avoid collision within the links, as well as with the environmental objects.\n          name: "MyCollisionDetection",\n          type: "CollisionAvoidance",\n          weight: 2,\n        },\n        jointLimit: {\n          name: "MyJointLimit",\n          type: "JointLimits",\n          weight: 5,\n        },\n        position: {\n          // The main objective that allows the pand hand to follow the position defined by the sphere transform control visual in the scene.\n          name: "MyPositionMatchObjective",\n          type: "PositionMatch",\n          link: "tool0",\n          weight: 15,\n        },\n        orientation: {\n          // The main objective that allows the pand hand to follow the orientation defined by the arrow transform control visual in the scene.\n          name: "MyOrientationMatchObjective",\n          type: "OrientationMatch",\n          link: "tool0",\n          weight: 10,\n        },\n      },\n      [\n        { value: 0.0, delta: 0.0 },\n        { value: 0.0, delta: 0.0 },\n        { value: 0.0, delta: 0.0 }, // Translational\n        { value: 0.0, delta: 0.0 },\n        { value: 0.0, delta: 0.0 },\n        { value: 0.0, delta: 0.0 }, // Rotational\n      ],\n      initialEnvShapes, // all the shapes passed in here will be considered as the static environmental shapes. This means that these shapes can not be modified through shapes_update in solve.\n      initialState\n    );\n    newSolver.computeAverageDistanceTable();\n    currentSolver = newSolver; //assign new solver to the current solver in use\n    // Assign the solver to the value\n    setLivelySolver(newSolver);\n    // Run solve to get a solved state\n    const newState = newSolver.solve(goal, {}, 0.0);\n    //console.log(newSolver.updates());\n    // Update the solver\'s current state\n    setRobotState(newState);\n    setVisualRobotState(newState);\n\n    return () => {\n      // Provide a function to clear previous values\n      setLivelySolver(null);\n      setRobotState(null);\n      setVisualRobotState(null);\n    };\n  }, [robot]); // Rerun this code if the robot changes\n\n  useEffect(() => {\n    let sinBox = [\n      {\n        Add: {\n          id: "env-box", // must be an unique id\n          shape: {\n            type: "Box", //can be \'Cylinder\', \'Capsule\', or \'Sphere\'\n            name: "box", // name can be arbitrary\n            frame: "world", // or \'world\'\n            physical: true, // physical collision\n            x: 0.25,\n            y: 0.25,\n            z: 0.25, // dimension of the box\n            localTransform: {\n              translation: [0.6, 0.05, 0.15],\n              rotation: [0.0, 0.0, 0.0, 1.0],\n            },\n          },\n        },\n      },\n      {\n        Move: {\n          id: "env-box",\n          transform: {\n            translation: [-0.5, 0.05, 0.15],\n            rotation: [0.0, 0.0, 0.0, 1.0],\n          },\n        },\n      },\n    ];\n    const updateSolver = () => {\n      if (livelySolver) {\n        const d = new Date();\n        //Math.sin(time / 1000) / 5 + 0.5\n        let time = d.getTime(); // Get the time in milliseconds\n        let newTranslation = [Math.sin(time / 1000) / 5 + -0.8, 0.05, 0.15];\n        sinBox[1].Move.transform.translation = newTranslation;\n\n        const newState = livelySolver.solve(\n          goal,\n          {},\n          time / 1000,\n          sinBox // compare to static environmental shapes, these shapes can be modified.\n        ); // Pass the new goal into solve function\n        // Update the solver\'s current state\n        setRobotState(newState);\n        setVisualRobotState(newState);\n        setEnvShapes([\n          {\n            type: "Box", //can be \'Cylinder\', \'Capsule\', or \'Sphere\'\n            name: "box", // name can be arbitrary\n            frame: "world", // or \'world\'\n            physical: true, // physical collision\n            x: 0.25,\n            y: 0.25,\n            z: 0.25, // dimension of the box\n            localTransform: {\n              translation: newTranslation,\n              rotation: [0.0, 0.0, 0.0, 1.0],\n            },\n          },\n        ]);\n      }\n\n      //console.log(goal);\n    };\n\n    const interval = setInterval(updateSolver, 1000 / 60);\n\n    return () => {\n      //setShapesUpdate(null);\n      setRobotState(null);\n      setVisualRobotState(null);\n      clearInterval(interval);\n    };\n  }, [livelySolver, shapesUpdate, goal]); // Update the solver 30fps},[robot])\n\n  return (\n    <div>\n      <RobotViewer\n        state={robotState}\n        links={livelySolver ? livelySolver.links : []}\n        shapes={envShapes}\n        showCollision={showCollision}\n        transformMode={transformMode}\n        transformControl={transformControl}\n        onMove={(id, source, worldTransform, localTransform) =>\n          updateTransform(id, localTransform)\n        }\n        activeEnvShapesTransform={activeShapesTransform}\n        levaOptions={{\n          showCollision: {\n            value: showCollision,\n            label: "Show Collisions",\n            onChange: (v) => setShowCollision(v),\n          },\n        }}\n      />\n      <Button\n        active={robot === "panda"}\n        onClick={() => setVisualRobotState(robotState)}\n      >\n        Update State\n      </Button>\n      <Button\n        active={transformMode === "translate"}\n        onClick={() => updateTransformMode()}\n      >\n        Position Match\n      </Button>\n      <Button\n        active={transformMode === "rotate"}\n        onClick={() => updateTransformMode()}\n      >\n        Orientation Match\n      </Button>\n\n      <Tree label="state" data={visualRobotState} />\n    </div>\n  );\n}\n')))))}p.isMDXComponent=!0}}]);