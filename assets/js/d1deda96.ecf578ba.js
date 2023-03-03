"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[6541],{3905:(e,t,a)=>{a.d(t,{Zo:()=>p,kt:()=>f});var n=a(67294);function r(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function o(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);t&&(n=n.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,n)}return a}function l(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?o(Object(a),!0).forEach((function(t){r(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):o(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function i(e,t){if(null==e)return{};var a,n,r=function(e,t){if(null==e)return{};var a,n,r={},o=Object.keys(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||(r[a]=e[a]);return r}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(r[a]=e[a])}return r}var s=n.createContext({}),u=function(e){var t=n.useContext(s),a=t;return e&&(a="function"==typeof e?e(t):l(l({},t),e)),a},p=function(e){var t=u(e.components);return n.createElement(s.Provider,{value:t},e.children)},c="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return n.createElement(n.Fragment,{},t)}},d=n.forwardRef((function(e,t){var a=e.components,r=e.mdxType,o=e.originalType,s=e.parentName,p=i(e,["components","mdxType","originalType","parentName"]),c=u(a),d=r,f=c["".concat(s,".").concat(d)]||c[d]||m[d]||o;return a?n.createElement(f,l(l({ref:t},p),{},{components:a})):n.createElement(f,l({ref:t},p))}));function f(e,t){var a=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var o=a.length,l=new Array(o);l[0]=d;var i={};for(var s in t)hasOwnProperty.call(t,s)&&(i[s]=t[s]);i.originalType=e,i[c]="string"==typeof e?e:r,l[1]=i;for(var u=2;u<o;u++)l[u]=a[u];return n.createElement.apply(null,l)}return n.createElement.apply(null,a)}d.displayName="MDXCreateElement"},85162:(e,t,a)=>{a.d(t,{Z:()=>l});var n=a(67294),r=a(86010);const o={tabItem:"tabItem_Ymn6"};function l(e){let{children:t,hidden:a,className:l}=e;return n.createElement("div",{role:"tabpanel",className:(0,r.Z)(o.tabItem,l),hidden:a},t)}},74866:(e,t,a)=>{a.d(t,{Z:()=>N});var n=a(87462),r=a(67294),o=a(86010),l=a(12466),i=a(16550),s=a(91980),u=a(67392),p=a(50012);function c(e){return function(e){return r.Children.map(e,(e=>{if((0,r.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))}(e).map((e=>{let{props:{value:t,label:a,attributes:n,default:r}}=e;return{value:t,label:a,attributes:n,default:r}}))}function m(e){const{values:t,children:a}=e;return(0,r.useMemo)((()=>{const e=t??c(a);return function(e){const t=(0,u.l)(e,((e,t)=>e.value===t.value));if(t.length>0)throw new Error(`Docusaurus error: Duplicate values "${t.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[t,a])}function d(e){let{value:t,tabValues:a}=e;return a.some((e=>e.value===t))}function f(e){let{queryString:t=!1,groupId:a}=e;const n=(0,i.k6)(),o=function(e){let{queryString:t=!1,groupId:a}=e;if("string"==typeof t)return t;if(!1===t)return null;if(!0===t&&!a)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return a??null}({queryString:t,groupId:a});return[(0,s._X)(o),(0,r.useCallback)((e=>{if(!o)return;const t=new URLSearchParams(n.location.search);t.set(o,e),n.replace({...n.location,search:t.toString()})}),[o,n])]}function b(e){const{defaultValue:t,queryString:a=!1,groupId:n}=e,o=m(e),[l,i]=(0,r.useState)((()=>function(e){let{defaultValue:t,tabValues:a}=e;if(0===a.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(t){if(!d({value:t,tabValues:a}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${t}" but none of its children has the corresponding value. Available values are: ${a.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return t}const n=a.find((e=>e.default))??a[0];if(!n)throw new Error("Unexpected error: 0 tabValues");return n.value}({defaultValue:t,tabValues:o}))),[s,u]=f({queryString:a,groupId:n}),[c,b]=function(e){let{groupId:t}=e;const a=function(e){return e?`docusaurus.tab.${e}`:null}(t),[n,o]=(0,p.Nk)(a);return[n,(0,r.useCallback)((e=>{a&&o.set(e)}),[a,o])]}({groupId:n}),k=(()=>{const e=s??c;return d({value:e,tabValues:o})?e:null})();(0,r.useLayoutEffect)((()=>{k&&i(k)}),[k]);return{selectedValue:l,selectValue:(0,r.useCallback)((e=>{if(!d({value:e,tabValues:o}))throw new Error(`Can't select invalid tab value=${e}`);i(e),u(e),b(e)}),[u,b,o]),tabValues:o}}var k=a(72389);const h={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};function y(e){let{className:t,block:a,selectedValue:i,selectValue:s,tabValues:u}=e;const p=[],{blockElementScrollPositionUntilNextRender:c}=(0,l.o5)(),m=e=>{const t=e.currentTarget,a=p.indexOf(t),n=u[a].value;n!==i&&(c(t),s(n))},d=e=>{let t=null;switch(e.key){case"Enter":m(e);break;case"ArrowRight":{const a=p.indexOf(e.currentTarget)+1;t=p[a]??p[0];break}case"ArrowLeft":{const a=p.indexOf(e.currentTarget)-1;t=p[a]??p[p.length-1];break}}t?.focus()};return r.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.Z)("tabs",{"tabs--block":a},t)},u.map((e=>{let{value:t,label:a,attributes:l}=e;return r.createElement("li",(0,n.Z)({role:"tab",tabIndex:i===t?0:-1,"aria-selected":i===t,key:t,ref:e=>p.push(e),onKeyDown:d,onClick:m},l,{className:(0,o.Z)("tabs__item",h.tabItem,l?.className,{"tabs__item--active":i===t})}),a??t)})))}function v(e){let{lazy:t,children:a,selectedValue:n}=e;if(a=Array.isArray(a)?a:[a],t){const e=a.find((e=>e.props.value===n));return e?(0,r.cloneElement)(e,{className:"margin-top--md"}):null}return r.createElement("div",{className:"margin-top--md"},a.map(((e,t)=>(0,r.cloneElement)(e,{key:t,hidden:e.props.value!==n}))))}function g(e){const t=b(e);return r.createElement("div",{className:(0,o.Z)("tabs-container",h.tabList)},r.createElement(y,(0,n.Z)({},e,t)),r.createElement(v,(0,n.Z)({},e,t)))}function N(e){const t=(0,k.Z)();return r.createElement(g,(0,n.Z)({key:String(t)},e))}},68743:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>p,contentTitle:()=>s,default:()=>f,frontMatter:()=>i,metadata:()=>u,toc:()=>c});var n=a(87462),r=(a(67294),a(3905)),o=a(74866),l=a(85162);const i={},s="State",u={unversionedId:"API/state",id:"API/state",title:"State",description:"The State object is the response back after calling solve.",source:"@site/docs/API/state.mdx",sourceDirName:"API",slug:"/API/state",permalink:"/lively/docs/API/state",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Capsule",permalink:"/lively/docs/API/Shapes/capsule"},next:{title:"Goals",permalink:"/lively/docs/API/Goals/goal"}},p={},c=[{value:"Import",id:"import",level:2},{value:"Declaration Example",id:"declaration-example",level:2}],m={toc:c},d="wrapper";function f(e){let{components:t,...a}=e;return(0,r.kt)(d,(0,n.Z)({},m,a,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"state"},"State"),(0,r.kt)("p",null,"The ",(0,r.kt)("inlineCode",{parentName:"p"},"State")," object is the response back after calling ",(0,r.kt)("a",{parentName:"p",href:"Solver/Methods/solve"},(0,r.kt)("inlineCode",{parentName:"a"},"solve")),".\nIt contains the state of the robot in terms of joint and frames, as well as some diagnostic information regarding ",(0,r.kt)("a",{parentName:"p",href:"../API/Info/proximityInfo"},(0,r.kt)("inlineCode",{parentName:"a"},"proximity"))," of various ",(0,r.kt)("a",{parentName:"p",href:"../API/Shapes/"},(0,r.kt)("inlineCode",{parentName:"a"},"shapes"))," and the ",(0,r.kt)("inlineCode",{parentName:"p"},"center_of_mass")," of the robot. "),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"For ",(0,r.kt)("a",{parentName:"p",href:"/lively/docs/API/Solver/initialization"},(0,r.kt)("inlineCode",{parentName:"a"},"Solver initialization")),", only ",(0,r.kt)("inlineCode",{parentName:"p"},"origin")," and ",(0,r.kt)("inlineCode",{parentName:"p"},"joints")," parameters have to be defined if user decided to provide a ",(0,r.kt)("inlineCode",{parentName:"p"},"State"),"object to the ",(0,r.kt)("inlineCode",{parentName:"p"},"initial_state")," parameter.")),(0,r.kt)("table",null,(0,r.kt)("thead",{parentName:"table"},(0,r.kt)("tr",{parentName:"thead"},(0,r.kt)("th",{parentName:"tr",align:null},"Parameter"),(0,r.kt)("th",{parentName:"tr",align:null},"Type"),(0,r.kt)("th",{parentName:"tr",align:null},"Optional"),(0,r.kt)("th",{parentName:"tr",align:null},"Description"))),(0,r.kt)("tbody",{parentName:"table"},(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"origin")),(0,r.kt)("td",{parentName:"tr",align:null},"isometry consisted of translation and rotation"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"The transform of the root of the robot. This data is also included in ",(0,r.kt)("inlineCode",{parentName:"td"},"frames"))),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"joints")),(0,r.kt)("td",{parentName:"tr",align:null},"lookup table of float indexed by a string key"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"A lookup table of the joint values for each movable joint")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"frames")),(0,r.kt)("td",{parentName:"tr",align:null},"lookup table of ",(0,r.kt)("a",{parentName:"td",href:"../API/Info/transformInfo"},(0,r.kt)("inlineCode",{parentName:"a"},"TransformInfo"))," indexed by string key"),(0,r.kt)("td",{parentName:"tr",align:null},"can be ignored for ",(0,r.kt)("a",{parentName:"td",href:"/lively/docs/API/Solver/initialization"},(0,r.kt)("inlineCode",{parentName:"a"},"Solver initialization"))),(0,r.kt)("td",{parentName:"tr",align:null},"A lookup table of each link\u2019s position in both world and local coordinates")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"proximity")),(0,r.kt)("td",{parentName:"tr",align:null},"list of ",(0,r.kt)("a",{parentName:"td",href:"../API/Info/proximityInfo"},(0,r.kt)("inlineCode",{parentName:"a"},"ProximityInfo"))),(0,r.kt)("td",{parentName:"tr",align:null},"can be ignored for ",(0,r.kt)("a",{parentName:"td",href:"/lively/docs/API/Solver/initialization"},(0,r.kt)("inlineCode",{parentName:"a"},"Solver initialization"))),(0,r.kt)("td",{parentName:"tr",align:null},"A vector of data representing pairwise proximity between the robot\u2019s parts and other robot parts and the environment. Each entry contains distance, as well as the closest points between the pair of colliders")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"center_of_mass")),(0,r.kt)("td",{parentName:"tr",align:null},"list of float"),(0,r.kt)("td",{parentName:"tr",align:null},"can be ignored for ",(0,r.kt)("a",{parentName:"td",href:"/lively/docs/API/Solver/initialization"},(0,r.kt)("inlineCode",{parentName:"a"},"Solver initialization"))),(0,r.kt)("td",{parentName:"tr",align:null},"A 3-vector representing the center of mass of the robot in the world frame")))),(0,r.kt)("h2",{id:"import"},"Import"),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"There is no need to import for Javascript.")),(0,r.kt)(o.Z,{mdxType:"Tabs"},(0,r.kt)(l.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},"  from lively import State\n"))),(0,r.kt)(l.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"},"  use lively::utils::state::State;\n")))),(0,r.kt)("h2",{id:"declaration-example"},"Declaration Example"),(0,r.kt)(o.Z,{mdxType:"Tabs"},(0,r.kt)(l.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-js"},'  let state = {origin:{translation:[0,0,0],rotation:[1,0,0,0]},joints:{"panda_joint1":0.0,"panda_joint2":0.0}}\n'))),(0,r.kt)(l.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},'  state = State(origin=Transform.identity(),joints={"panda_joint1":0.0,"panda_joint2":0.0})\n'))),(0,r.kt)(l.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"}," let iso = Isometry3::from_parts(\n      Translation3::new(\n          0.6497281999999998,\n          -0.24972819999999987,\n          0.050000000000000044,\n      ),\n      UnitQuaternion::from_quaternion(Quaternion::new(\n          0.0,\n          0.0,\n          -0.7069999677447771,\n          0.7072135784958345,\n      )),\n  );\n  let mut joints: HashMap<String, f64> = HashMap::new();\n  let mut frames: HashMap<String,TransformInfo> = HashMap::new();\n  let mut proximities = Vec::new();\n  let mut center_of_mass: Vector3<f64>::new();\n  let state = State::new(iso, joints, frames, proximities, center_of_mass);\n")))))}f.isMDXComponent=!0}}]);