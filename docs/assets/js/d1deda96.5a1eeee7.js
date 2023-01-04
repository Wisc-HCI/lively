"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[6541],{3905:(e,t,n)=>{n.d(t,{Zo:()=>u,kt:()=>f});var a=n(7294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function l(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function i(e,t){if(null==e)return{};var n,a,r=function(e,t){if(null==e)return{};var n,a,r={},o=Object.keys(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var s=a.createContext({}),p=function(e){var t=a.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):l(l({},t),e)),n},u=function(e){var t=p(e.components);return a.createElement(s.Provider,{value:t},e.children)},m="mdxType",c={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},d=a.forwardRef((function(e,t){var n=e.components,r=e.mdxType,o=e.originalType,s=e.parentName,u=i(e,["components","mdxType","originalType","parentName"]),m=p(n),d=r,f=m["".concat(s,".").concat(d)]||m[d]||c[d]||o;return n?a.createElement(f,l(l({ref:t},u),{},{components:n})):a.createElement(f,l({ref:t},u))}));function f(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var o=n.length,l=new Array(o);l[0]=d;var i={};for(var s in t)hasOwnProperty.call(t,s)&&(i[s]=t[s]);i.originalType=e,i[m]="string"==typeof e?e:r,l[1]=i;for(var p=2;p<o;p++)l[p]=n[p];return a.createElement.apply(null,l)}return a.createElement.apply(null,n)}d.displayName="MDXCreateElement"},5162:(e,t,n)=>{n.d(t,{Z:()=>l});var a=n(7294),r=n(6010);const o="tabItem_Ymn6";function l(e){let{children:t,hidden:n,className:l}=e;return a.createElement("div",{role:"tabpanel",className:(0,r.Z)(o,l),hidden:n},t)}},5488:(e,t,n)=>{n.d(t,{Z:()=>d});var a=n(7462),r=n(7294),o=n(6010),l=n(2389),i=n(7392),s=n(7094),p=n(2466);const u="tabList__CuJ",m="tabItem_LNqP";function c(e){const{lazy:t,block:n,defaultValue:l,values:c,groupId:d,className:f}=e,b=r.Children.map(e.children,(e=>{if((0,r.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)})),h=c??b.map((e=>{let{props:{value:t,label:n,attributes:a}}=e;return{value:t,label:n,attributes:a}})),k=(0,i.l)(h,((e,t)=>e.value===t.value));if(k.length>0)throw new Error(`Docusaurus error: Duplicate values "${k.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`);const y=null===l?l:l??b.find((e=>e.props.default))?.props.value??b[0].props.value;if(null!==y&&!h.some((e=>e.value===y)))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${y}" but none of its children has the corresponding value. Available values are: ${h.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);const{tabGroupChoices:v,setTabGroupChoices:g}=(0,s.U)(),[N,w]=(0,r.useState)(y),I=[],{blockElementScrollPositionUntilNextRender:T}=(0,p.o5)();if(null!=d){const e=v[d];null!=e&&e!==N&&h.some((t=>t.value===e))&&w(e)}const j=e=>{const t=e.currentTarget,n=I.indexOf(t),a=h[n].value;a!==N&&(T(t),w(a),null!=d&&g(d,String(a)))},O=e=>{let t=null;switch(e.key){case"Enter":j(e);break;case"ArrowRight":{const n=I.indexOf(e.currentTarget)+1;t=I[n]??I[0];break}case"ArrowLeft":{const n=I.indexOf(e.currentTarget)-1;t=I[n]??I[I.length-1];break}}t?.focus()};return r.createElement("div",{className:(0,o.Z)("tabs-container",u)},r.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.Z)("tabs",{"tabs--block":n},f)},h.map((e=>{let{value:t,label:n,attributes:l}=e;return r.createElement("li",(0,a.Z)({role:"tab",tabIndex:N===t?0:-1,"aria-selected":N===t,key:t,ref:e=>I.push(e),onKeyDown:O,onClick:j},l,{className:(0,o.Z)("tabs__item",m,l?.className,{"tabs__item--active":N===t})}),n??t)}))),t?(0,r.cloneElement)(b.filter((e=>e.props.value===N))[0],{className:"margin-top--md"}):r.createElement("div",{className:"margin-top--md"},b.map(((e,t)=>(0,r.cloneElement)(e,{key:t,hidden:e.props.value!==N})))))}function d(e){const t=(0,l.Z)();return r.createElement(c,(0,a.Z)({key:String(t)},e))}},8743:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>u,contentTitle:()=>s,default:()=>d,frontMatter:()=>i,metadata:()=>p,toc:()=>m});var a=n(7462),r=(n(7294),n(3905)),o=n(5488),l=n(5162);const i={},s="State",p={unversionedId:"API/state",id:"API/state",title:"State",description:"The State object is the response back after calling solve.",source:"@site/docs/API/state.mdx",sourceDirName:"API",slug:"/API/state",permalink:"/docs/API/state",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Capsule",permalink:"/docs/API/Shapes/capsule"},next:{title:"Goals",permalink:"/docs/API/Goals/goal"}},u={},m=[{value:"Import",id:"import",level:2}],c={toc:m};function d(e){let{components:t,...n}=e;return(0,r.kt)("wrapper",(0,a.Z)({},c,n,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"state"},"State"),(0,r.kt)("p",null,"The ",(0,r.kt)("inlineCode",{parentName:"p"},"State")," object is the response back after calling ",(0,r.kt)("a",{parentName:"p",href:"Solver/Methods/solve"},(0,r.kt)("inlineCode",{parentName:"a"},"solve")),".\nIt contains the state of the robot in terms of joint and frames, as well as some diagnostic information regarding ",(0,r.kt)("a",{parentName:"p",href:"../API/Info/proximityInfo"},(0,r.kt)("inlineCode",{parentName:"a"},"proximity"))," of various ",(0,r.kt)("a",{parentName:"p",href:"../API/Shapes/"},(0,r.kt)("inlineCode",{parentName:"a"},"shapes"))," and the ",(0,r.kt)("inlineCode",{parentName:"p"},"center_of_mass")," of the robot. "),(0,r.kt)("table",null,(0,r.kt)("thead",{parentName:"table"},(0,r.kt)("tr",{parentName:"thead"},(0,r.kt)("th",{parentName:"tr",align:null},"Parameter"),(0,r.kt)("th",{parentName:"tr",align:null},"Type"),(0,r.kt)("th",{parentName:"tr",align:null},"Description"))),(0,r.kt)("tbody",{parentName:"table"},(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"origin")),(0,r.kt)("td",{parentName:"tr",align:null},"isometry consisted of translation and rotation"),(0,r.kt)("td",{parentName:"tr",align:null},"The transform of the root of the robot. This data is also included in ",(0,r.kt)("inlineCode",{parentName:"td"},"frames"))),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"joints")),(0,r.kt)("td",{parentName:"tr",align:null},"lookup table(string, float)"),(0,r.kt)("td",{parentName:"tr",align:null},"A lookup table of the joint values for each movable joint")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"frames")),(0,r.kt)("td",{parentName:"tr",align:null},"lookup table(string, ",(0,r.kt)("a",{parentName:"td",href:"../API/Info/transformInfo"},(0,r.kt)("inlineCode",{parentName:"a"},"TransformInfo")),")"),(0,r.kt)("td",{parentName:"tr",align:null},"A lookup table of each link\u2019s position in both world and local coordinates")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"proximity")),(0,r.kt)("td",{parentName:"tr",align:null},"list of ",(0,r.kt)("a",{parentName:"td",href:"../API/Info/proximityInfo"},(0,r.kt)("inlineCode",{parentName:"a"},"ProximityInfo"))),(0,r.kt)("td",{parentName:"tr",align:null},"A vector of data representing pairwise proximity between the robot\u2019s parts and other robot parts and the environment. Each entry contains distance, as well as the closest points between the pair of colliders")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"center_of_mass")),(0,r.kt)("td",{parentName:"tr",align:null},"list of float"),(0,r.kt)("td",{parentName:"tr",align:null},"A 3-vector representing the center of mass of the robot in the world frame")))),(0,r.kt)(o.Z,{mdxType:"Tabs"},(0,r.kt)(l.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-js"},'  let state = {origin:{translation:[0,0,0],rotation:[1,0,0,0]},joints:{"panda_joint1":0.0,"panda_joint2":0.0}}\n'))),(0,r.kt)(l.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},'  state = State(origin=Transform.identity(),joints={"panda_joint1":0.0,"panda_joint2":0.0})\n'))),(0,r.kt)(l.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"}," let iso = Isometry3::from_parts(\n      Translation3::new(\n          0.6497281999999998,\n          -0.24972819999999987,\n          0.050000000000000044,\n      ),\n      UnitQuaternion::from_quaternion(Quaternion::new(\n          0.0,\n          0.0,\n          -0.7069999677447771,\n          0.7072135784958345,\n      )),\n  );\n  let mut joints: HashMap<String, f64> = HashMap::new();\n  let mut frames: HashMap<String,TransformInfo> = HashMap::new();\n  let mut proximities = Vec::new();\n  let mut center_of_mass: Vector3<f64>::new();\n  let state = State::new(iso, joints, frames, proximities, center_of_mass);\n")))),(0,r.kt)("h2",{id:"import"},"Import"))}d.isMDXComponent=!0}}]);