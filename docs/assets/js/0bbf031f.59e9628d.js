"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[345],{3905:(e,t,n)=>{n.d(t,{Zo:()=>u,kt:()=>d});var o=n(7294);function a(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function r(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);t&&(o=o.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,o)}return n}function s(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?r(Object(n),!0).forEach((function(t){a(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):r(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,o,a=function(e,t){if(null==e)return{};var n,o,a={},r=Object.keys(e);for(o=0;o<r.length;o++)n=r[o],t.indexOf(n)>=0||(a[n]=e[n]);return a}(e,t);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);for(o=0;o<r.length;o++)n=r[o],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(a[n]=e[n])}return a}var i=o.createContext({}),c=function(e){var t=o.useContext(i),n=t;return e&&(n="function"==typeof e?e(t):s(s({},t),e)),n},u=function(e){var t=c(e.components);return o.createElement(i.Provider,{value:t},e.children)},v="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return o.createElement(o.Fragment,{},t)}},p=o.forwardRef((function(e,t){var n=e.components,a=e.mdxType,r=e.originalType,i=e.parentName,u=l(e,["components","mdxType","originalType","parentName"]),v=c(n),p=a,d=v["".concat(i,".").concat(p)]||v[p]||m[p]||r;return n?o.createElement(d,s(s({ref:t},u),{},{components:n})):o.createElement(d,s({ref:t},u))}));function d(e,t){var n=arguments,a=t&&t.mdxType;if("string"==typeof e||a){var r=n.length,s=new Array(r);s[0]=p;var l={};for(var i in t)hasOwnProperty.call(t,i)&&(l[i]=t[i]);l.originalType=e,l[v]="string"==typeof e?e:a,s[1]=l;for(var c=2;c<r;c++)s[c]=n[c];return o.createElement.apply(null,s)}return o.createElement.apply(null,n)}p.displayName="MDXCreateElement"},5162:(e,t,n)=>{n.d(t,{Z:()=>s});var o=n(7294),a=n(6010);const r="tabItem_Ymn6";function s(e){let{children:t,hidden:n,className:s}=e;return o.createElement("div",{role:"tabpanel",className:(0,a.Z)(r,s),hidden:n},t)}},5488:(e,t,n)=>{n.d(t,{Z:()=>p});var o=n(7462),a=n(7294),r=n(6010),s=n(2389),l=n(7392),i=n(7094),c=n(2466);const u="tabList__CuJ",v="tabItem_LNqP";function m(e){const{lazy:t,block:n,defaultValue:s,values:m,groupId:p,className:d}=e,b=a.Children.map(e.children,(e=>{if((0,a.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)})),h=m??b.map((e=>{let{props:{value:t,label:n,attributes:o}}=e;return{value:t,label:n,attributes:o}})),f=(0,l.l)(h,((e,t)=>e.value===t.value));if(f.length>0)throw new Error(`Docusaurus error: Duplicate values "${f.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`);const y=null===s?s:s??b.find((e=>e.props.default))?.props.value??b[0].props.value;if(null!==y&&!h.some((e=>e.value===y)))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${y}" but none of its children has the corresponding value. Available values are: ${h.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);const{tabGroupChoices:g,setTabGroupChoices:w}=(0,i.U)(),[S,j]=(0,a.useState)(y),k=[],{blockElementScrollPositionUntilNextRender:O}=(0,c.o5)();if(null!=p){const e=g[p];null!=e&&e!==S&&h.some((t=>t.value===e))&&j(e)}const T=e=>{const t=e.currentTarget,n=k.indexOf(t),o=h[n].value;o!==S&&(O(t),j(o),null!=p&&w(p,String(o)))},x=e=>{let t=null;switch(e.key){case"Enter":T(e);break;case"ArrowRight":{const n=k.indexOf(e.currentTarget)+1;t=k[n]??k[0];break}case"ArrowLeft":{const n=k.indexOf(e.currentTarget)-1;t=k[n]??k[k.length-1];break}}t?.focus()};return a.createElement("div",{className:(0,r.Z)("tabs-container",u)},a.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,r.Z)("tabs",{"tabs--block":n},d)},h.map((e=>{let{value:t,label:n,attributes:s}=e;return a.createElement("li",(0,o.Z)({role:"tab",tabIndex:S===t?0:-1,"aria-selected":S===t,key:t,ref:e=>k.push(e),onKeyDown:x,onClick:T},s,{className:(0,r.Z)("tabs__item",v,s?.className,{"tabs__item--active":S===t})}),n??t)}))),t?(0,a.cloneElement)(b.filter((e=>e.props.value===S))[0],{className:"margin-top--md"}):a.createElement("div",{className:"margin-top--md"},b.map(((e,t)=>(0,a.cloneElement)(e,{key:t,hidden:e.props.value!==S})))))}function p(e){const t=(0,s.Z)();return a.createElement(m,(0,o.Z)({key:String(t)},e))}},5360:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>u,contentTitle:()=>i,default:()=>p,frontMatter:()=>l,metadata:()=>c,toc:()=>v});var o=n(7462),a=(n(7294),n(3905)),r=n(5488),s=n(5162);const l={},i="Environment Modeling",c={unversionedId:"Tutorials/environment",id:"Tutorials/environment",title:"Environment Modeling",description:"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!",source:"@site/docs/Tutorials/environment.md",sourceDirName:"Tutorials",slug:"/Tutorials/environment",permalink:"/docs/Tutorials/environment",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"tutorials",previous:{title:"Adding Liveliness",permalink:"/docs/Tutorials/liveliness"},next:{title:"Customized Usage",permalink:"/docs/category/customized-usage"}},u={},v=[],m={toc:v};function p(e){let{components:t,...n}=e;return(0,a.kt)("wrapper",(0,o.Z)({},m,n,{components:t,mdxType:"MDXLayout"}),(0,a.kt)("h1",{id:"environment-modeling"},"Environment Modeling"),(0,a.kt)("p",null,(0,a.kt)("em",{parentName:"p"},"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!")),(0,a.kt)(r.Z,{mdxType:"Tabs"},(0,a.kt)(s.Z,{value:"jsx",label:"Live",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-jsx",metastring:"live",live:!0},"function InitializationExample(props) {\n  const [livelySolver, setLivelySolver] = useState(null);\n  const [robot, setRobot] = useState('panda');\n  const [robotState, setRobotState] = useState(null)\n  \n  useEffect(()=>{\n      /* \n      Given that we are showing this example in a declarative\n      react context, we need to use the useEffect hook to execute\n      imperative (sequential) code. That means that if you are\n      writing standard javascript, your code will look like the\n      contents of the \"init\" function.\n      * Note also that the \"init\" function is async. This is\n      because the lively library is built on web assembly (WASM),\n      which needs to be imported asynchronously.\n      */\n      const init = async ()=>{\n          // Initialize the lively package (WASM)\n          await lively.init();\n          // Instantiate a new solver\n          const newSolver = new lively.Solver(\n              urdfs[robot], // The urdf of the robot\n              {\n                  'smoothness': {  // An example objective (smoothness macro)\n                      name: 'MySmoothnessObjective',\n                      type: 'SmoothnessMacro',\n                      weight: 5\n                  }\n              }\n          );\n          // Assign the solver to the value\n          setLivelySolver(newSolver)\n          // Run solve to get a solved state\n          const newState = newSolver.solve({},{},0.0);\n          // Update the solver's current state\n          setRobotState(newState)\n      }\n      init();\n      \n      return ()=>{\n          // Provide a function to clear previous values\n          setLivelySolver(null);\n          setRobotState(null);\n      }\n  },[robot]) // Rerun this code if the robot changes\n\nreturn (\n  <div>\n     <Button active={robot==='panda'} onClick={()=>setRobot('panda')}>Panda</Button>\n     <Button active={robot==='ur3e'} onClick={()=>setRobot('ur3e')}>UR3e</Button>\n     <RobotViewer state={robotState} links={livelySolver ? livelySolver.links : []}/>\n     <Tree label='state' data={robotState}/>\n  </div>\n);\n}\n"))),(0,a.kt)(s.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-js"},"import init, {Solver} from 'lively';\n\nasync function start() {\n  // Initialize the lively package (WASM)\n  await init();\n  // Instantiate a new solver\n  let solver = new Solver(\n      \"<?xml version='1.0' ?><robot name='panda'>...</robot>\", // Full urdf as a string\n      {\n          'smoothness': {  // An example objective (smoothness macro)\n              name: 'MySmoothnessObjective',\n              type: 'SmoothnessMacro',\n              weight: 5\n          }\n      }\n  );\n  // Run solve to get a solved state\n  let state = solver.solve({},{},0.0);\n  // Log the initial state\n  console.log(state)\n}\n\n// Could be executed from anywhere that supports async actions\nstart();\n\n"))),(0,a.kt)(s.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-py"},'from lively import Solver, SmoothnessMacroObjective\n\n# Instantiate a new solver\nsolver = Solver(\n  urdf=\'<?xml version="1.0" ?><robot name="panda">...</robot>\', # Full urdf as a string\n  objectives={\n      # An example objective (smoothness macro)\n      "smoothness":SmoothnessMacroObjective(name="MySmoothnessObjective",weight=5)\n  }\n)\n\n# Run solve to get a solved state\nstate = solver.solve({},{},0.0)\n# Log the initial state\nprint(state)\n'))),(0,a.kt)(s.Z,{value:"rs",label:"Rust",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-rust"},'use lively::lively::Solver;\nuse lively::objectives::core::base::SmoothnessMacroObjective;\nuse lively::objectives::objective::Objective;\nuse std::collections::HashMap;\n\n// Create a map of objectives\nlet mut objectives: HashMap<String,Objective> = HashMap::new();\n// Add a Smoothness Macro Objective\nobjectives.insert(\n  "smoothness".into(),\n  // An example objective (smoothness macro)\n  Objective::SmoothnessMacro(SmoothnessMacroObjective::new("MySmoothnessObjective",5.0))\n);\n\n// Instantiate a new solver struct\nlet mut solver = Solver::new(\n  urdf:\'<?xml version="1.0" ?><robot name="panda">...</robot>\', // Full urdf as a string\n  objectives\n);\n\n// Run solve to get a solved state\nlet state = solver.solve(\n  HashMap::new(), \n  HashMap::new(), \n  0.0, \n  None\n);\n// Log the initial state\nprintln!("{:?}",state);\n')))))}p.isMDXComponent=!0}}]);