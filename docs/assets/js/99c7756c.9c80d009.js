"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[2570],{3905:(e,t,n)=>{n.d(t,{Zo:()=>u,kt:()=>b});var a=n(7294);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function r(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?r(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):r(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function s(e,t){if(null==e)return{};var n,a,o=function(e,t){if(null==e)return{};var n,a,o={},r=Object.keys(e);for(a=0;a<r.length;a++)n=r[a],t.indexOf(n)>=0||(o[n]=e[n]);return o}(e,t);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);for(a=0;a<r.length;a++)n=r[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(o[n]=e[n])}return o}var l=a.createContext({}),c=function(e){var t=a.useContext(l),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},u=function(e){var t=c(e.components);return a.createElement(l.Provider,{value:t},e.children)},v="mdxType",p={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},m=a.forwardRef((function(e,t){var n=e.components,o=e.mdxType,r=e.originalType,l=e.parentName,u=s(e,["components","mdxType","originalType","parentName"]),v=c(n),m=o,b=v["".concat(l,".").concat(m)]||v[m]||p[m]||r;return n?a.createElement(b,i(i({ref:t},u),{},{components:n})):a.createElement(b,i({ref:t},u))}));function b(e,t){var n=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var r=n.length,i=new Array(r);i[0]=m;var s={};for(var l in t)hasOwnProperty.call(t,l)&&(s[l]=t[l]);s.originalType=e,s[v]="string"==typeof e?e:o,i[1]=s;for(var c=2;c<r;c++)i[c]=n[c];return a.createElement.apply(null,i)}return a.createElement.apply(null,n)}m.displayName="MDXCreateElement"},5162:(e,t,n)=>{n.d(t,{Z:()=>i});var a=n(7294),o=n(6010);const r="tabItem_Ymn6";function i(e){let{children:t,hidden:n,className:i}=e;return a.createElement("div",{role:"tabpanel",className:(0,o.Z)(r,i),hidden:n},t)}},5488:(e,t,n)=>{n.d(t,{Z:()=>m});var a=n(7462),o=n(7294),r=n(6010),i=n(2389),s=n(7392),l=n(7094),c=n(2466);const u="tabList__CuJ",v="tabItem_LNqP";function p(e){const{lazy:t,block:n,defaultValue:i,values:p,groupId:m,className:b}=e,d=o.Children.map(e.children,(e=>{if((0,o.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)})),h=p??d.map((e=>{let{props:{value:t,label:n,attributes:a}}=e;return{value:t,label:n,attributes:a}})),f=(0,s.l)(h,((e,t)=>e.value===t.value));if(f.length>0)throw new Error(`Docusaurus error: Duplicate values "${f.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`);const y=null===i?i:i??d.find((e=>e.props.default))?.props.value??d[0].props.value;if(null!==y&&!h.some((e=>e.value===y)))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${y}" but none of its children has the corresponding value. Available values are: ${h.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);const{tabGroupChoices:g,setTabGroupChoices:w}=(0,l.U)(),[S,j]=(0,o.useState)(y),k=[],{blockElementScrollPositionUntilNextRender:O}=(0,c.o5)();if(null!=m){const e=g[m];null!=e&&e!==S&&h.some((t=>t.value===e))&&j(e)}const T=e=>{const t=e.currentTarget,n=k.indexOf(t),a=h[n].value;a!==S&&(O(t),j(a),null!=m&&w(m,String(a)))},x=e=>{let t=null;switch(e.key){case"Enter":T(e);break;case"ArrowRight":{const n=k.indexOf(e.currentTarget)+1;t=k[n]??k[0];break}case"ArrowLeft":{const n=k.indexOf(e.currentTarget)-1;t=k[n]??k[k.length-1];break}}t?.focus()};return o.createElement("div",{className:(0,r.Z)("tabs-container",u)},o.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,r.Z)("tabs",{"tabs--block":n},b)},h.map((e=>{let{value:t,label:n,attributes:i}=e;return o.createElement("li",(0,a.Z)({role:"tab",tabIndex:S===t?0:-1,"aria-selected":S===t,key:t,ref:e=>k.push(e),onKeyDown:x,onClick:T},i,{className:(0,r.Z)("tabs__item",v,i?.className,{"tabs__item--active":S===t})}),n??t)}))),t?(0,o.cloneElement)(d.filter((e=>e.props.value===S))[0],{className:"margin-top--md"}):o.createElement("div",{className:"margin-top--md"},d.map(((e,t)=>(0,o.cloneElement)(e,{key:t,hidden:e.props.value!==S})))))}function m(e){const t=(0,i.Z)();return o.createElement(p,(0,a.Z)({key:String(t)},e))}},5781:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>u,contentTitle:()=>l,default:()=>m,frontMatter:()=>s,metadata:()=>c,toc:()=>v});var a=n(7462),o=(n(7294),n(3905)),r=n(5488),i=n(5162);const s={},l="Basic Initialization",c={unversionedId:"Tutorials/initialization",id:"Tutorials/initialization",title:"Basic Initialization",description:"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!",source:"@site/docs/Tutorials/initialization.md",sourceDirName:"Tutorials",slug:"/Tutorials/initialization",permalink:"/docs/Tutorials/initialization",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"tutorials",previous:{title:"Basic Usage",permalink:"/docs/category/basic-usage"},next:{title:"Advanced Initialization",permalink:"/docs/Tutorials/advanced_initialization"}},u={},v=[],p={toc:v};function m(e){let{components:t,...n}=e;return(0,o.kt)("wrapper",(0,a.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,o.kt)("h1",{id:"basic-initialization"},"Basic Initialization"),(0,o.kt)("p",null,(0,o.kt)("em",{parentName:"p"},"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!")),(0,o.kt)(r.Z,{mdxType:"Tabs"},(0,o.kt)(i.Z,{value:"jsx",label:"Live",mdxType:"TabItem"},(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-jsx",metastring:"live",live:!0},"function InitializationExample(props) {\n  const [livelySolver, setLivelySolver] = useState(null);\n  const [robot, setRobot] = useState('panda');\n  const [robotState, setRobotState] = useState(null)\n  \n  useEffect(()=>{\n      /* \n      Given that we are showing this example in a declarative\n      react context, we need to use the useEffect hook to execute\n      imperative (sequential) code. That means that if you are\n      writing standard javascript, your code will look like the\n      contents of the \"init\" function.\n      * Note also that the \"init\" function is async. This is\n      because the lively library is built on web assembly (WASM),\n      which needs to be imported asynchronously.\n      */\n      const init = async ()=>{\n          // Initialize the lively package (WASM)\n          await lively.init();\n          // Instantiate a new solver\n          const newSolver = new lively.Solver(\n              urdfs[robot], // The urdf of the robot\n              {\n                  'smoothness': {  // An example objective (smoothness macro)\n                      name: 'MySmoothnessObjective',\n                      type: 'SmoothnessMacro',\n                      weight: 5\n                  }\n              }\n          );\n          // Assign the solver to the value\n          setLivelySolver(newSolver)\n          // Run solve to get a solved state\n          const newState = newSolver.solve({},{},0.0);\n          // Update the solver's current state\n          setRobotState(newState)\n      }\n      init();\n      \n      return ()=>{\n          // Provide a function to clear previous values\n          setLivelySolver(null);\n          setRobotState(null);\n      }\n  },[robot]) // Rerun this code if the robot changes\n\nreturn (\n  <div>\n     <Button active={robot==='panda'} onClick={()=>setRobot('panda')}>Panda</Button>\n     <Button active={robot==='ur3e'} onClick={()=>setRobot('ur3e')}>UR3e</Button>\n     <RobotViewer state={robotState} links={livelySolver ? livelySolver.links : []}/>\n     <Tree label='state' data={robotState}/>\n  </div>\n);\n}\n"))),(0,o.kt)(i.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-js"},"import init, {Solver} from 'lively';\n\nasync function start() {\n  // Initialize the lively package (WASM)\n  await init();\n  // Instantiate a new solver\n  let solver = new Solver(\n      \"<?xml version='1.0' ?><robot name='panda'>...</robot>\", // Full urdf as a string\n      {\n          'smoothness': {  // An example objective (smoothness macro)\n              name: 'MySmoothnessObjective',\n              type: 'SmoothnessMacro',\n              weight: 5\n          }\n      }\n  );\n  // Run solve to get a solved state\n  let state = solver.solve({},{},0.0);\n  // Log the initial state\n  console.log(state)\n}\n\n// Could be executed from anywhere that supports async actions\nstart();\n\n"))),(0,o.kt)(i.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-py"},'from lively import Solver, SmoothnessMacroObjective\n\n# Instantiate a new solver\nsolver = Solver(\n  urdf=\'<?xml version="1.0" ?><robot name="panda">...</robot>\', # Full urdf as a string\n  objectives={\n      # An example objective (smoothness macro)\n      "smoothness":SmoothnessMacroObjective(name="MySmoothnessObjective",weight=5)\n  }\n)\n\n# Run solve to get a solved state\nstate = solver.solve({},{},0.0)\n# Log the initial state\nprint(state)\n'))),(0,o.kt)(i.Z,{value:"rs",label:"Rust",mdxType:"TabItem"},(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-rust"},'use lively::lively::Solver;\nuse lively::objectives::core::base::SmoothnessMacroObjective;\nuse lively::objectives::objective::Objective;\nuse std::collections::HashMap;\n\n// Create a map of objectives\nlet mut objectives: HashMap<String,Objective> = HashMap::new();\n// Add a Smoothness Macro Objective\nobjectives.insert(\n  "smoothness".into(),\n  // An example objective (smoothness macro)\n  Objective::SmoothnessMacro(SmoothnessMacroObjective::new("MySmoothnessObjective",5.0))\n);\n\n// Instantiate a new solver struct\nlet mut solver = Solver::new(\n  urdf:\'<?xml version="1.0" ?><robot name="panda">...</robot>\', // Full urdf as a string\n  objectives\n);\n\n// Run solve to get a solved state\nlet state = solver.solve(\n  HashMap::new(), \n  HashMap::new(), \n  0.0, \n  None\n);\n// Log the initial state\nprintln!("{:?}",state);\n')))))}m.isMDXComponent=!0}}]);