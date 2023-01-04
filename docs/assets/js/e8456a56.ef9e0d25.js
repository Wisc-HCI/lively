"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[3421],{3905:(e,t,a)=>{a.d(t,{Zo:()=>u,kt:()=>v});var r=a(7294);function n(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function l(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,r)}return a}function o(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?l(Object(a),!0).forEach((function(t){n(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):l(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function i(e,t){if(null==e)return{};var a,r,n=function(e,t){if(null==e)return{};var a,r,n={},l=Object.keys(e);for(r=0;r<l.length;r++)a=l[r],t.indexOf(a)>=0||(n[a]=e[a]);return n}(e,t);if(Object.getOwnPropertySymbols){var l=Object.getOwnPropertySymbols(e);for(r=0;r<l.length;r++)a=l[r],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(n[a]=e[a])}return n}var s=r.createContext({}),p=function(e){var t=r.useContext(s),a=t;return e&&(a="function"==typeof e?e(t):o(o({},t),e)),a},u=function(e){var t=p(e.components);return r.createElement(s.Provider,{value:t},e.children)},c="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},d=r.forwardRef((function(e,t){var a=e.components,n=e.mdxType,l=e.originalType,s=e.parentName,u=i(e,["components","mdxType","originalType","parentName"]),c=p(a),d=n,v=c["".concat(s,".").concat(d)]||c[d]||m[d]||l;return a?r.createElement(v,o(o({ref:t},u),{},{components:a})):r.createElement(v,o({ref:t},u))}));function v(e,t){var a=arguments,n=t&&t.mdxType;if("string"==typeof e||n){var l=a.length,o=new Array(l);o[0]=d;var i={};for(var s in t)hasOwnProperty.call(t,s)&&(i[s]=t[s]);i.originalType=e,i[c]="string"==typeof e?e:n,o[1]=i;for(var p=2;p<l;p++)o[p]=a[p];return r.createElement.apply(null,o)}return r.createElement.apply(null,a)}d.displayName="MDXCreateElement"},5162:(e,t,a)=>{a.d(t,{Z:()=>o});var r=a(7294),n=a(6010);const l="tabItem_Ymn6";function o(e){let{children:t,hidden:a,className:o}=e;return r.createElement("div",{role:"tabpanel",className:(0,n.Z)(l,o),hidden:a},t)}},5488:(e,t,a)=>{a.d(t,{Z:()=>d});var r=a(7462),n=a(7294),l=a(6010),o=a(2389),i=a(7392),s=a(7094),p=a(2466);const u="tabList__CuJ",c="tabItem_LNqP";function m(e){const{lazy:t,block:a,defaultValue:o,values:m,groupId:d,className:v}=e,b=n.Children.map(e.children,(e=>{if((0,n.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)})),k=m??b.map((e=>{let{props:{value:t,label:a,attributes:r}}=e;return{value:t,label:a,attributes:r}})),g=(0,i.l)(k,((e,t)=>e.value===t.value));if(g.length>0)throw new Error(`Docusaurus error: Duplicate values "${g.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`);const f=null===o?o:o??b.find((e=>e.props.default))?.props.value??b[0].props.value;if(null!==f&&!k.some((e=>e.value===f)))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${f}" but none of its children has the corresponding value. Available values are: ${k.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);const{tabGroupChoices:N,setTabGroupChoices:h}=(0,s.U)(),[y,T]=(0,n.useState)(f),j=[],{blockElementScrollPositionUntilNextRender:_}=(0,p.o5)();if(null!=d){const e=N[d];null!=e&&e!==y&&k.some((t=>t.value===e))&&T(e)}const w=e=>{const t=e.currentTarget,a=j.indexOf(t),r=k[a].value;r!==y&&(_(t),T(r),null!=d&&h(d,String(r)))},x=e=>{let t=null;switch(e.key){case"Enter":w(e);break;case"ArrowRight":{const a=j.indexOf(e.currentTarget)+1;t=j[a]??j[0];break}case"ArrowLeft":{const a=j.indexOf(e.currentTarget)-1;t=j[a]??j[j.length-1];break}}t?.focus()};return n.createElement("div",{className:(0,l.Z)("tabs-container",u)},n.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,l.Z)("tabs",{"tabs--block":a},v)},k.map((e=>{let{value:t,label:a,attributes:o}=e;return n.createElement("li",(0,r.Z)({role:"tab",tabIndex:y===t?0:-1,"aria-selected":y===t,key:t,ref:e=>j.push(e),onKeyDown:x,onClick:w},o,{className:(0,l.Z)("tabs__item",c,o?.className,{"tabs__item--active":y===t})}),a??t)}))),t?(0,n.cloneElement)(b.filter((e=>e.props.value===y))[0],{className:"margin-top--md"}):n.createElement("div",{className:"margin-top--md"},b.map(((e,t)=>(0,n.cloneElement)(e,{key:t,hidden:e.props.value!==y})))))}function d(e){const t=(0,o.Z)();return n.createElement(m,(0,r.Z)({key:String(t)},e))}},7135:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>u,contentTitle:()=>s,default:()=>d,frontMatter:()=>i,metadata:()=>p,toc:()=>c});var r=a(7462),n=(a(7294),a(3905)),l=a(5488),o=a(5162);const i={},s="Properties",p={unversionedId:"API/Solver/Properties/index",id:"API/Solver/Properties/index",title:"Properties",description:"| Property | Description |",source:"@site/docs/API/Solver/Properties/index.mdx",sourceDirName:"API/Solver/Properties",slug:"/API/Solver/Properties/",permalink:"/docs/API/Solver/Properties/",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Collision Normalization",permalink:"/docs/API/Solver/Methods/collision_normalization"},next:{title:"Shapes",permalink:"/docs/API/Shapes/"}},u={},c=[{value:"Retriving current <code>state</code>",id:"retriving-current-state",level:2},{value:"Retriving current <code>objectives</code>",id:"retriving-current-objectives",level:2},{value:"Modifying current <code>objectives</code>",id:"modifying-current-objectives",level:2},{value:"Retriving current <code>goals</code>",id:"retriving-current-goals",level:2},{value:"Modifying current <code>goals</code>",id:"modifying-current-goals",level:2}],m={toc:c};function d(e){let{components:t,...a}=e;return(0,n.kt)("wrapper",(0,r.Z)({},m,a,{components:t,mdxType:"MDXLayout"}),(0,n.kt)("h1",{id:"properties"},"Properties"),(0,n.kt)("table",null,(0,n.kt)("thead",{parentName:"table"},(0,n.kt)("tr",{parentName:"thead"},(0,n.kt)("th",{parentName:"tr",align:null},"Property"),(0,n.kt)("th",{parentName:"tr",align:null},"Description"))),(0,n.kt)("tbody",{parentName:"table"},(0,n.kt)("tr",{parentName:"tbody"},(0,n.kt)("td",{parentName:"tr",align:null},(0,n.kt)("inlineCode",{parentName:"td"},"robot_model")),(0,n.kt)("td",{parentName:"tr",align:null},"A set of information and properties of a robot")),(0,n.kt)("tr",{parentName:"tbody"},(0,n.kt)("td",{parentName:"tr",align:null},(0,n.kt)("inlineCode",{parentName:"td"},"vars")),(0,n.kt)("td",{parentName:"tr",align:null},"A set of useful features within the optimization, including history, joint, and link information")),(0,n.kt)("tr",{parentName:"tbody"},(0,n.kt)("td",{parentName:"tr",align:null},(0,n.kt)("inlineCode",{parentName:"td"},"lower_bounds")),(0,n.kt)("td",{parentName:"tr",align:null},"A vector representing the lower bounds in the optimization vector. This is derived from the robot base limits and joint limits")),(0,n.kt)("tr",{parentName:"tbody"},(0,n.kt)("td",{parentName:"tr",align:null},(0,n.kt)("inlineCode",{parentName:"td"},"upper_bounds")),(0,n.kt)("td",{parentName:"tr",align:null},"A vector representing the upper bounds in the optimization vector. This is derived from the robot base limits, as well as joint limits")),(0,n.kt)("tr",{parentName:"tbody"},(0,n.kt)("td",{parentName:"tr",align:null},(0,n.kt)("inlineCode",{parentName:"td"},"objective_set")),(0,n.kt)("td",{parentName:"tr",align:null},"A lookup table containing all of the current ",(0,n.kt)("a",{parentName:"td",href:"../../Objectives/"},(0,n.kt)("inlineCode",{parentName:"a"},"objectives"))," in use")),(0,n.kt)("tr",{parentName:"tbody"},(0,n.kt)("td",{parentName:"tr",align:null},(0,n.kt)("inlineCode",{parentName:"td"},"max_retries")),(0,n.kt)("td",{parentName:"tr",align:null},"The maximum number of randomly initialized rounds allowed for each invocation of ",(0,n.kt)("a",{parentName:"td",href:"../Methods/solve"},(0,n.kt)("inlineCode",{parentName:"a"},"solve")))),(0,n.kt)("tr",{parentName:"tbody"},(0,n.kt)("td",{parentName:"tr",align:null},(0,n.kt)("inlineCode",{parentName:"td"},"max_iterations")),(0,n.kt)("td",{parentName:"tr",align:null},"The number of maximum iterations per round within the optimization")))),(0,n.kt)("h2",{id:"retriving-current-state"},"Retriving current ",(0,n.kt)("a",{parentName:"h2",href:"../../state"},(0,n.kt)("inlineCode",{parentName:"a"},"state"))),(0,n.kt)("p",null,"Returns the solution ",(0,n.kt)("a",{parentName:"p",href:"../../state"},(0,n.kt)("inlineCode",{parentName:"a"},"state"))," from the last invocation of ",(0,n.kt)("a",{parentName:"p",href:"../Methods/solve"},(0,n.kt)("inlineCode",{parentName:"a"},"solve"))),(0,n.kt)(l.Z,{mdxType:"Tabs"},(0,n.kt)(o.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-js"},"let current_state = solver.current_state();\n"))),(0,n.kt)(o.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-py"},"current_state = solver.get_current_state();\n"))),(0,n.kt)(o.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-rust"},"let current_state = solver.get_current_state();\n")))),(0,n.kt)("h2",{id:"retriving-current-objectives"},"Retriving current ",(0,n.kt)("a",{parentName:"h2",href:"../../Objectives/"},(0,n.kt)("inlineCode",{parentName:"a"},"objectives"))),(0,n.kt)("p",null,"Returns the current ",(0,n.kt)("a",{parentName:"p",href:"../../Objectives/"},(0,n.kt)("inlineCode",{parentName:"a"},"objectives"))," as a lookup table"),(0,n.kt)(l.Z,{mdxType:"Tabs"},(0,n.kt)(o.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-js"},"let current_objectives = solver.objectives();\n"))),(0,n.kt)(o.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-py"},"current_objectives = solver.get_objectives();\n"))),(0,n.kt)(o.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-rust"},"let current_state = solver::get_objectives();\n")))),(0,n.kt)("h2",{id:"modifying-current-objectives"},"Modifying current ",(0,n.kt)("a",{parentName:"h2",href:"../../Objectives/"},(0,n.kt)("inlineCode",{parentName:"a"},"objectives"))),(0,n.kt)("p",null,"Replace the current ",(0,n.kt)("a",{parentName:"p",href:"../../Objectives/"},(0,n.kt)("inlineCode",{parentName:"a"},"objectives"))," with new ",(0,n.kt)("a",{parentName:"p",href:"../../Objectives/"},(0,n.kt)("inlineCode",{parentName:"a"},"objectives"))," lookup table"),(0,n.kt)(l.Z,{mdxType:"Tabs"},(0,n.kt)(o.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-js"},"solver.objectives = new_objectives;\n"))),(0,n.kt)(o.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-py"},"solver.objectives = new_objectives;\n"))),(0,n.kt)(o.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-rust"},"solver.set_objectives(new_objectives);\n")))),(0,n.kt)("h2",{id:"retriving-current-goals"},"Retriving current ",(0,n.kt)("a",{parentName:"h2",href:"../../Goals/goal"},(0,n.kt)("inlineCode",{parentName:"a"},"goals"))),(0,n.kt)("p",null,"Returns the current ",(0,n.kt)("a",{parentName:"p",href:"../../Goals/goal"},(0,n.kt)("inlineCode",{parentName:"a"},"goals"))," as a lookup table"),(0,n.kt)(l.Z,{mdxType:"Tabs"},(0,n.kt)(o.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-js"},"let current_goals = solver.current_goals();\n"))),(0,n.kt)(o.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-py"},"current_goals = solver.get_current_goals();\n"))),(0,n.kt)(o.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-rust"},"let current_goals = solver.get_current_goals();\n")))),(0,n.kt)("h2",{id:"modifying-current-goals"},"Modifying current ",(0,n.kt)("a",{parentName:"h2",href:"../../Goals/goal"},(0,n.kt)("inlineCode",{parentName:"a"},"goals"))),(0,n.kt)("p",null,"Replace the current ",(0,n.kt)("a",{parentName:"p",href:"../../Goals/goal"},(0,n.kt)("inlineCode",{parentName:"a"},"goals"))," with new ",(0,n.kt)("a",{parentName:"p",href:"../../Goals/goal"},(0,n.kt)("inlineCode",{parentName:"a"},"goals"))," lookup table"),(0,n.kt)(l.Z,{mdxType:"Tabs"},(0,n.kt)(o.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-js"},"solver.goals = new_goals;\n"))),(0,n.kt)(o.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-py"},"solver.goals = new_goals;\n"))),(0,n.kt)(o.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,n.kt)("pre",null,(0,n.kt)("code",{parentName:"pre",className:"language-rust"},"solver.set_goals(new_goals);\n")))))}d.isMDXComponent=!0}}]);