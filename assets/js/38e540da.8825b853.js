"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[4394],{3905:(e,t,n)=>{n.d(t,{Zo:()=>d,kt:()=>h});var a=n(67294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function l(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function i(e,t){if(null==e)return{};var n,a,r=function(e,t){if(null==e)return{};var n,a,r={},o=Object.keys(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var s=a.createContext({}),p=function(e){var t=a.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):l(l({},t),e)),n},d=function(e){var t=p(e.components);return a.createElement(s.Provider,{value:t},e.children)},u="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},c=a.forwardRef((function(e,t){var n=e.components,r=e.mdxType,o=e.originalType,s=e.parentName,d=i(e,["components","mdxType","originalType","parentName"]),u=p(n),c=r,h=u["".concat(s,".").concat(c)]||u[c]||m[c]||o;return n?a.createElement(h,l(l({ref:t},d),{},{components:n})):a.createElement(h,l({ref:t},d))}));function h(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var o=n.length,l=new Array(o);l[0]=c;var i={};for(var s in t)hasOwnProperty.call(t,s)&&(i[s]=t[s]);i.originalType=e,i[u]="string"==typeof e?e:r,l[1]=i;for(var p=2;p<o;p++)l[p]=n[p];return a.createElement.apply(null,l)}return a.createElement.apply(null,n)}c.displayName="MDXCreateElement"},85162:(e,t,n)=>{n.d(t,{Z:()=>l});var a=n(67294),r=n(86010);const o="tabItem_Ymn6";function l(e){let{children:t,hidden:n,className:l}=e;return a.createElement("div",{role:"tabpanel",className:(0,r.Z)(o,l),hidden:n},t)}},74866:(e,t,n)=>{n.d(t,{Z:()=>w});var a=n(83117),r=n(67294),o=n(86010),l=n(12466),i=n(16550),s=n(91980),p=n(67392),d=n(50012);function u(e){return function(e){return r.Children.map(e,(e=>{if((0,r.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))}(e).map((e=>{let{props:{value:t,label:n,attributes:a,default:r}}=e;return{value:t,label:n,attributes:a,default:r}}))}function m(e){const{values:t,children:n}=e;return(0,r.useMemo)((()=>{const e=t??u(n);return function(e){const t=(0,p.l)(e,((e,t)=>e.value===t.value));if(t.length>0)throw new Error(`Docusaurus error: Duplicate values "${t.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[t,n])}function c(e){let{value:t,tabValues:n}=e;return n.some((e=>e.value===t))}function h(e){let{queryString:t=!1,groupId:n}=e;const a=(0,i.k6)(),o=function(e){let{queryString:t=!1,groupId:n}=e;if("string"==typeof t)return t;if(!1===t)return null;if(!0===t&&!n)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return n??null}({queryString:t,groupId:n});return[(0,s._X)(o),(0,r.useCallback)((e=>{if(!o)return;const t=new URLSearchParams(a.location.search);t.set(o,e),a.replace({...a.location,search:t.toString()})}),[o,a])]}function v(e){const{defaultValue:t,queryString:n=!1,groupId:a}=e,o=m(e),[l,i]=(0,r.useState)((()=>function(e){let{defaultValue:t,tabValues:n}=e;if(0===n.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(t){if(!c({value:t,tabValues:n}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${t}" but none of its children has the corresponding value. Available values are: ${n.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return t}const a=n.find((e=>e.default))??n[0];if(!a)throw new Error("Unexpected error: 0 tabValues");return a.value}({defaultValue:t,tabValues:o}))),[s,p]=h({queryString:n,groupId:a}),[u,v]=function(e){let{groupId:t}=e;const n=function(e){return e?`docusaurus.tab.${e}`:null}(t),[a,o]=(0,d.Nk)(n);return[a,(0,r.useCallback)((e=>{n&&o.set(e)}),[n,o])]}({groupId:a}),f=(()=>{const e=s??u;return c({value:e,tabValues:o})?e:null})();(0,r.useLayoutEffect)((()=>{f&&i(f)}),[f]);return{selectedValue:l,selectValue:(0,r.useCallback)((e=>{if(!c({value:e,tabValues:o}))throw new Error(`Can't select invalid tab value=${e}`);i(e),p(e),v(e)}),[p,v,o]),tabValues:o}}var f=n(72389);const b="tabList__CuJ",y="tabItem_LNqP";function k(e){let{className:t,block:n,selectedValue:i,selectValue:s,tabValues:p}=e;const d=[],{blockElementScrollPositionUntilNextRender:u}=(0,l.o5)(),m=e=>{const t=e.currentTarget,n=d.indexOf(t),a=p[n].value;a!==i&&(u(t),s(a))},c=e=>{let t=null;switch(e.key){case"Enter":m(e);break;case"ArrowRight":{const n=d.indexOf(e.currentTarget)+1;t=d[n]??d[0];break}case"ArrowLeft":{const n=d.indexOf(e.currentTarget)-1;t=d[n]??d[d.length-1];break}}t?.focus()};return r.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.Z)("tabs",{"tabs--block":n},t)},p.map((e=>{let{value:t,label:n,attributes:l}=e;return r.createElement("li",(0,a.Z)({role:"tab",tabIndex:i===t?0:-1,"aria-selected":i===t,key:t,ref:e=>d.push(e),onKeyDown:c,onClick:m},l,{className:(0,o.Z)("tabs__item",y,l?.className,{"tabs__item--active":i===t})}),n??t)})))}function g(e){let{lazy:t,children:n,selectedValue:a}=e;if(n=Array.isArray(n)?n:[n],t){const e=n.find((e=>e.props.value===a));return e?(0,r.cloneElement)(e,{className:"margin-top--md"}):null}return r.createElement("div",{className:"margin-top--md"},n.map(((e,t)=>(0,r.cloneElement)(e,{key:t,hidden:e.props.value!==a}))))}function N(e){const t=v(e);return r.createElement("div",{className:(0,o.Z)("tabs-container",b)},r.createElement(k,(0,a.Z)({},e,t)),r.createElement(g,(0,a.Z)({},e,t)))}function w(e){const t=(0,f.Z)();return r.createElement(N,(0,a.Z)({key:String(t)},e))}},73175:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>d,contentTitle:()=>s,default:()=>c,frontMatter:()=>i,metadata:()=>p,toc:()=>u});var a=n(83117),r=(n(67294),n(3905)),o=n(74866),l=n(85162);const i={},s="Solving",p={unversionedId:"API/Solver/Methods/solve",id:"API/Solver/Methods/solve",title:"Solving",description:"The Solver class has a solve method that represents the core functionality of the Lively interface. At a high level, it returns a fully-filled state object and accepts the following fields:",source:"@site/docs/API/Solver/Methods/solve.mdx",sourceDirName:"API/Solver/Methods",slug:"/API/Solver/Methods/solve",permalink:"/lively/docs/API/Solver/Methods/solve",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Methods",permalink:"/lively/docs/category/methods"},next:{title:"Resetting",permalink:"/lively/docs/API/Solver/Methods/reset"}},d={},u=[{value:"Shape Update",id:"shape-update",level:2},{value:"Example for adding a new environmental shape",id:"example-for-adding-a-new-environmental-shape",level:3},{value:"Example for moving an existing environmental shape",id:"example-for-moving-an-existing-environmental-shape",level:3},{value:"Example for deleting an existing environmental shape",id:"example-for-deleting-an-existing-environmental-shape",level:3}],m={toc:u};function c(e){let{components:t,...n}=e;return(0,r.kt)("wrapper",(0,a.Z)({},m,n,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"solving"},"Solving"),(0,r.kt)("p",null,"The ",(0,r.kt)("inlineCode",{parentName:"p"},"Solver")," class has a ",(0,r.kt)("inlineCode",{parentName:"p"},"solve")," method that represents the core functionality of the Lively interface. At a high level, it returns a fully-filled ",(0,r.kt)("a",{parentName:"p",href:"../../state"},(0,r.kt)("inlineCode",{parentName:"a"},"state"))," object and accepts the following fields:"),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"The ",(0,r.kt)("strong",{parentName:"p"},"optional")," field can be left empty if the no specification is needed. The ",(0,r.kt)("inlineCode",{parentName:"p"},"Solver")," will just use the default values for the optional parameters.")),(0,r.kt)("table",null,(0,r.kt)("thead",{parentName:"table"},(0,r.kt)("tr",{parentName:"thead"},(0,r.kt)("th",{parentName:"tr",align:null},"Field"),(0,r.kt)("th",{parentName:"tr",align:null},"Type"),(0,r.kt)("th",{parentName:"tr",align:null},"Optional"),(0,r.kt)("th",{parentName:"tr",align:null},"Description"))),(0,r.kt)("tbody",{parentName:"table"},(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"goals")),(0,r.kt)("td",{parentName:"tr",align:null},"look-up table of ",(0,r.kt)("a",{parentName:"td",href:"../../Goals/goal"},(0,r.kt)("inlineCode",{parentName:"a"},"goal"))," indexed by a string key"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"The key of the look-up table should match with that of the ",(0,r.kt)("a",{parentName:"td",href:"../../Objectives"},(0,r.kt)("inlineCode",{parentName:"a"},"objectives"))," to which the ",(0,r.kt)("inlineCode",{parentName:"td"},"goals")," are corresponded.")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"weights")),(0,r.kt)("td",{parentName:"tr",align:null},"look-up table of float value indexed by a string key"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"The key of the look-up table should match with that of the ",(0,r.kt)("a",{parentName:"td",href:"../../Objectives"},(0,r.kt)("inlineCode",{parentName:"a"},"objectives"))," to which the weights are corresponded.")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"time")),(0,r.kt)("td",{parentName:"tr",align:null},"float"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"The current time stamp. It is advised to always pass in a increasing time stamp for the solver to have a more consistent behavior.")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"shape_update")),(0,r.kt)("td",{parentName:"tr",align:null},"list of ",(0,r.kt)("inlineCode",{parentName:"td"},"ShapeUpdate")," objects"),(0,r.kt)("td",{parentName:"tr",align:null},"yes"),(0,r.kt)("td",{parentName:"tr",align:null},"see below")))),(0,r.kt)("h2",{id:"shape-update"},"Shape Update"),(0,r.kt)("p",null,"The ",(0,r.kt)("inlineCode",{parentName:"p"},"shape_update")," parameter in ",(0,r.kt)("inlineCode",{parentName:"p"},"solve")," takes in a list of ",(0,r.kt)("inlineCode",{parentName:"p"},"ShapeUpdate")," objects specifying the environmental shapes to be introduced, modified, and removed.\nThe ",(0,r.kt)("inlineCode",{parentName:"p"},"ShapeUpdate")," object allows for three functionalities or types: ",(0,r.kt)("inlineCode",{parentName:"p"},"Add"),", ",(0,r.kt)("inlineCode",{parentName:"p"},"Move"),", and ",(0,r.kt)("inlineCode",{parentName:"p"},"Delete")," each corresponding to the tasks mentioned above. For more information please see the example below."),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"The ",(0,r.kt)("a",{parentName:"p",href:"../../Shapes/"},(0,r.kt)("inlineCode",{parentName:"a"},"shape"))," objects passed in the ",(0,r.kt)("inlineCode",{parentName:"p"},"shapes")," parameter in ",(0,r.kt)("inlineCode",{parentName:"p"},"solver")," ",(0,r.kt)("a",{parentName:"p",href:"../initialization"},(0,r.kt)("inlineCode",{parentName:"a"},"initialization"))," are static and can not be moved or deleted later. Only the shapes that are introduced (",(0,r.kt)("inlineCode",{parentName:"p"},"Add"),") to the environment in ",(0,r.kt)("inlineCode",{parentName:"p"},"ShapeUpdate")," can be moved or deleted by ",(0,r.kt)("inlineCode",{parentName:"p"},"Move")," and ",(0,r.kt)("inlineCode",{parentName:"p"},"delete"),".")),(0,r.kt)("h3",{id:"example-for-adding-a-new-environmental-shape"},"Example for adding a new environmental shape"),(0,r.kt)("p",null,"When a new shape is introduced to the environment, the ",(0,r.kt)("inlineCode",{parentName:"p"},"id")," associated with the new shape needs to be unique.\nThe ",(0,r.kt)("inlineCode",{parentName:"p"},"id")," will be used to identify the shape when a ",(0,r.kt)("inlineCode",{parentName:"p"},"Move")," or ",(0,r.kt)("inlineCode",{parentName:"p"},"Delete")," is performed on the same shape."),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"If another shape is added to the environment with the same ",(0,r.kt)("inlineCode",{parentName:"p"},"id"),", the new shape will replace the old shape with the same ",(0,r.kt)("inlineCode",{parentName:"p"},"id"),".")),(0,r.kt)(o.Z,{mdxType:"Tabs"},(0,r.kt)(l.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-js"},'const addCube = [\n  Add: {\n    // This is an Add ShapeUpdate functionality or type. This introduce a new Cube with "env-box" as the id to the environment.\n    id: "env-box", // must be an unique id\n    shape: {\n      type: "Box", //can be \'Cylinder\', \'Capsule\', or \'Sphere\'\n      name: "box", // name can be arbitrary\n      frame: "world", // frame name\n      physical: true, // physical collision\n      x: 0.25,\n      y: 0.25,\n      z: 0.25, // dimension of the box\n      localTransform: {\n        translation: [0.6, 0.05, 0.15],\n        rotation: [0.0, 0.0, 0.0, 1.0],\n      },\n    },\n  },\n];\nconst d = new Date();\nlet time = d.getTime(); // Get the time in milliseconds\ncurrentSolver.solve({}, {}, time / 1000, addCube);\n'))),(0,r.kt)(l.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},'add_box = BoxShape(name="Table",frame="world",physical=True,x=2,y=1,z=1.2,local_transform=Transform.isometry())\n# This is an Add ShapeUpdate functionality or type. This introduce a new Cube with "box_1" as the id to the environment.\nshape_update = [pyShapeUpdate.Add(id = "box_1", shape = add_box)]\n# pass the shape_update to shape_update parameter\ncurrentSolver.solve(shape_update = shape_update)\n'))),(0,r.kt)(l.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"},"  let iso = Isometry3::from_parts( //declaring transform consisted of translation and rotation\n      Translation3::new(\n          0.6, 0.05, 0.15\n      ),\n      UnitQuaternion::from_quaternion(Quaternion::new(\n          0.0, 0.0, 0.0, 1.0\n      )),\n  );\n  let add_box = Shape::Box(BoxShape::new(//can be 'Cylinder', 'Capsule', or 'Sphere'\n      \"conveyorCollisionShapeBase\".to_string(),// name can be arbitrary\n      \"world\".to_string(),// frame name\n      true, // physical collision\n      1.0,\n      1.1,\n      1.7,\n      iso, // transform\n  ));\n  let shape_update: Vec<ShapeUpdate> = vec![\n      // This is an Add ShapeUpdate functionality or type. This introduce a new Cube with \"box_1\" as the id to the environment.\n      ShapeUpdate::Add {\n          id: box_1.to_string(),// must be an unique id\n          shape: add_box.clone(),\n      }\n  ];\n  solver.solve(\n      goals.clone(),\n      weights.clone(),\n      0.0,\n      Some(shape_update.clone()), //shape update\n      None,\n    );\n")))),(0,r.kt)("h3",{id:"example-for-moving-an-existing-environmental-shape"},"Example for moving an existing environmental shape"),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"If a wrong ",(0,r.kt)("inlineCode",{parentName:"p"},"id")," is used, the solver will not change the transformation of any shape.")),(0,r.kt)(o.Z,{mdxType:"Tabs"},(0,r.kt)(l.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-js"},'const moveCube = [\n  Move: {\n    // This is a Move ShapeUpdate functionality or type. This modifies the transform of the Cube introduced by Add in the environment\n    id: "env-box", // must be identical to the id created in the shape is added\n    transform: {// the shape will be moved to this transformation\n      translation: [1.6, 1.05, 1.15],\n      rotation: [0.0, 1.0, 0.0, 1.0],\n    },\n  },\n];\nconst d = new Date();\nlet time = d.getTime(); // Get the time in milliseconds\ncurrentSolver.solve({}, {}, time / 1000, moveCube);\n'))),(0,r.kt)(l.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},'# This is a Move ShapeUpdate functionality or type. This move the Cube with "box_1" as the id in the environment.\nshape_update = [pyShapeUpdate.Move(id =  "box_1", pose = local_transform(translation = [1.6, 1.05, 1.15],\nrotation = [0.0, 1.0, 0.0, 1.0]))]\n# pass the shape_update to shape_update parameter\ncurrentSolver.solve(shape_update = shape_update)\n'))),(0,r.kt)(l.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"},'let move_iso = Isometry3::from_parts( //declaring transform consisted of translation and rotation\n    Translation3::new(\n        1.6, 1.05, 1.15\n    ),\n    UnitQuaternion::from_quaternion(Quaternion::new(\n        0.0, 1.0, 0.0, 1.0\n    )),\n);\nlet shape_update: Vec<ShapeUpdate> = vec![\n    // This is an move ShapeUpdate functionality or type. This moves the Cube with "box_1" as the id to the environment.\n    ShapeUpdate::Move {\n        id: box_1.to_string(),// must be an unique id\n        pose: move_iso.to_string()\n    }\n];\nsolver.solve(\n    goals.clone(),\n    weights.clone(),\n    0.0,\n    Some(shape_update.clone()), //shape update\n    None,\n  );\n')))),(0,r.kt)("h3",{id:"example-for-deleting-an-existing-environmental-shape"},"Example for deleting an existing environmental shape"),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"If a wrong ",(0,r.kt)("inlineCode",{parentName:"p"},"id")," is used, the solver will not delete any shape.")),(0,r.kt)(o.Z,{mdxType:"Tabs"},(0,r.kt)(l.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-js"},'const deleteCube = [Delete: "env-box"];// This is a delete ShapeUpdate functionality or type. This deletes the Cube introduced by Add in the environment.\nconst d = new Date();\nlet time = d.getTime(); // Get the time in milliseconds\ncurrentSolver.solve({}, {}, time / 1000, deleteCube);\n'))),(0,r.kt)(l.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},'# This is a Delete ShapeUpdate functionality or type. This delete the Cube with "box_1" as the id from the environment.\nshape_update = [pyShapeUpdate.Delete(id =  "box_1", pose = local_transform(translation = [1.6, 1.05, 1.15],\nrotation = [0.0, 1.0, 0.0, 1.0]))]\n# pass the shape_update to shape_update parameter\ncurrentSolver.solve(shape_update = shape_update)\n'))),(0,r.kt)(l.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"},'let shape_update: Vec<ShapeUpdate> = vec![\nShapeUpdate::Delete{// This is an delete ShapeUpdate functionality or type. This deletes the Cube with "box_1" as the id from the environment.\n id : "box_1".to_string(),\n}];\nsolver.solve(\n  goals.clone(),\n  weights.clone(),\n  0.0,\n  Some(shape_update.clone()), //shape update\n  None,\n);\n')))))}c.isMDXComponent=!0}}]);