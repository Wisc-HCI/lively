"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[769],{3905:(e,t,n)=>{n.d(t,{Zo:()=>c,kt:()=>b});var r=n(67294);function a(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){a(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,r,a=function(e,t){if(null==e)return{};var n,r,a={},o=Object.keys(e);for(r=0;r<o.length;r++)n=o[r],t.indexOf(n)>=0||(a[n]=e[n]);return a}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(r=0;r<o.length;r++)n=o[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(a[n]=e[n])}return a}var s=r.createContext({}),u=function(e){var t=r.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},c=function(e){var t=u(e.components);return r.createElement(s.Provider,{value:t},e.children)},p="mdxType",d={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},m=r.forwardRef((function(e,t){var n=e.components,a=e.mdxType,o=e.originalType,s=e.parentName,c=l(e,["components","mdxType","originalType","parentName"]),p=u(n),m=a,b=p["".concat(s,".").concat(m)]||p[m]||d[m]||o;return n?r.createElement(b,i(i({ref:t},c),{},{components:n})):r.createElement(b,i({ref:t},c))}));function b(e,t){var n=arguments,a=t&&t.mdxType;if("string"==typeof e||a){var o=n.length,i=new Array(o);i[0]=m;var l={};for(var s in t)hasOwnProperty.call(t,s)&&(l[s]=t[s]);l.originalType=e,l[p]="string"==typeof e?e:a,i[1]=l;for(var u=2;u<o;u++)i[u]=n[u];return r.createElement.apply(null,i)}return r.createElement.apply(null,n)}m.displayName="MDXCreateElement"},85162:(e,t,n)=>{n.d(t,{Z:()=>i});var r=n(67294),a=n(86010);const o="tabItem_Ymn6";function i(e){let{children:t,hidden:n,className:i}=e;return r.createElement("div",{role:"tabpanel",className:(0,a.Z)(o,i),hidden:n},t)}},74866:(e,t,n)=>{n.d(t,{Z:()=>w});var r=n(83117),a=n(67294),o=n(86010),i=n(12466),l=n(16550),s=n(91980),u=n(67392),c=n(50012);function p(e){return function(e){return a.Children.map(e,(e=>{if((0,a.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))}(e).map((e=>{let{props:{value:t,label:n,attributes:r,default:a}}=e;return{value:t,label:n,attributes:r,default:a}}))}function d(e){const{values:t,children:n}=e;return(0,a.useMemo)((()=>{const e=t??p(n);return function(e){const t=(0,u.l)(e,((e,t)=>e.value===t.value));if(t.length>0)throw new Error(`Docusaurus error: Duplicate values "${t.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[t,n])}function m(e){let{value:t,tabValues:n}=e;return n.some((e=>e.value===t))}function b(e){let{queryString:t=!1,groupId:n}=e;const r=(0,l.k6)(),o=function(e){let{queryString:t=!1,groupId:n}=e;if("string"==typeof t)return t;if(!1===t)return null;if(!0===t&&!n)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return n??null}({queryString:t,groupId:n});return[(0,s._X)(o),(0,a.useCallback)((e=>{if(!o)return;const t=new URLSearchParams(r.location.search);t.set(o,e),r.replace({...r.location,search:t.toString()})}),[o,r])]}function f(e){const{defaultValue:t,queryString:n=!1,groupId:r}=e,o=d(e),[i,l]=(0,a.useState)((()=>function(e){let{defaultValue:t,tabValues:n}=e;if(0===n.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(t){if(!m({value:t,tabValues:n}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${t}" but none of its children has the corresponding value. Available values are: ${n.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return t}const r=n.find((e=>e.default))??n[0];if(!r)throw new Error("Unexpected error: 0 tabValues");return r.value}({defaultValue:t,tabValues:o}))),[s,u]=b({queryString:n,groupId:r}),[p,f]=function(e){let{groupId:t}=e;const n=function(e){return e?`docusaurus.tab.${e}`:null}(t),[r,o]=(0,c.Nk)(n);return[r,(0,a.useCallback)((e=>{n&&o.set(e)}),[n,o])]}({groupId:r}),h=(()=>{const e=s??p;return m({value:e,tabValues:o})?e:null})();(0,a.useLayoutEffect)((()=>{h&&l(h)}),[h]);return{selectedValue:i,selectValue:(0,a.useCallback)((e=>{if(!m({value:e,tabValues:o}))throw new Error(`Can't select invalid tab value=${e}`);l(e),u(e),f(e)}),[u,f,o]),tabValues:o}}var h=n(72389);const v="tabList__CuJ",g="tabItem_LNqP";function k(e){let{className:t,block:n,selectedValue:l,selectValue:s,tabValues:u}=e;const c=[],{blockElementScrollPositionUntilNextRender:p}=(0,i.o5)(),d=e=>{const t=e.currentTarget,n=c.indexOf(t),r=u[n].value;r!==l&&(p(t),s(r))},m=e=>{let t=null;switch(e.key){case"Enter":d(e);break;case"ArrowRight":{const n=c.indexOf(e.currentTarget)+1;t=c[n]??c[0];break}case"ArrowLeft":{const n=c.indexOf(e.currentTarget)-1;t=c[n]??c[c.length-1];break}}t?.focus()};return a.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.Z)("tabs",{"tabs--block":n},t)},u.map((e=>{let{value:t,label:n,attributes:i}=e;return a.createElement("li",(0,r.Z)({role:"tab",tabIndex:l===t?0:-1,"aria-selected":l===t,key:t,ref:e=>c.push(e),onKeyDown:m,onClick:d},i,{className:(0,o.Z)("tabs__item",g,i?.className,{"tabs__item--active":l===t})}),n??t)})))}function y(e){let{lazy:t,children:n,selectedValue:r}=e;if(n=Array.isArray(n)?n:[n],t){const e=n.find((e=>e.props.value===r));return e?(0,a.cloneElement)(e,{className:"margin-top--md"}):null}return a.createElement("div",{className:"margin-top--md"},n.map(((e,t)=>(0,a.cloneElement)(e,{key:t,hidden:e.props.value!==r}))))}function N(e){const t=f(e);return a.createElement("div",{className:(0,o.Z)("tabs-container",v)},a.createElement(k,(0,r.Z)({},e,t)),a.createElement(y,(0,r.Z)({},e,t)))}function w(e){const t=(0,h.Z)();return a.createElement(N,(0,r.Z)({key:String(t)},e))}},18090:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>c,contentTitle:()=>s,default:()=>m,frontMatter:()=>l,metadata:()=>u,toc:()=>p});var r=n(83117),a=(n(67294),n(3905)),o=n(74866),i=n(85162);const l={},s="Objective",u={unversionedId:"API/Objectives/index",id:"API/Objectives/index",title:"Objective",description:"Lively allows for a wide range of robot with which users program robot motion. These 24 properties, which serve as building blocks for defining the behavior and motion of the robot, fit into five categories:",source:"@site/docs/API/Objectives/index.mdx",sourceDirName:"API/Objectives",slug:"/API/Objectives/",permalink:"/lively/docs/API/Objectives/",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"CollisionSettingInfo",permalink:"/lively/docs/API/Info/collisionSettingInfo"},next:{title:"Base",permalink:"/lively/docs/API/Objectives/base"}},c={},p=[{value:"Customizing the priority of the Objective",id:"customizing-the-priority-of-the-objective",level:2},{value:"Import",id:"import",level:2}],d={toc:p};function m(e){let{components:t,...n}=e;return(0,a.kt)("wrapper",(0,r.Z)({},d,n,{components:t,mdxType:"MDXLayout"}),(0,a.kt)("h1",{id:"objective"},"Objective"),(0,a.kt)("p",null,"Lively allows for a wide range of robot with which users program robot motion. These 24 properties, which serve as building blocks for defining the behavior and motion of the robot, fit into five categories:\n",(0,a.kt)("inlineCode",{parentName:"p"},"Base"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Bounding"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Matchting"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Mirroring"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Liveliness"),", and ",(0,a.kt)("inlineCode",{parentName:"p"},"Forces")," listed blow. "),(0,a.kt)("table",null,(0,a.kt)("thead",{parentName:"table"},(0,a.kt)("tr",{parentName:"thead"},(0,a.kt)("th",{parentName:"tr",align:null},"Category"),(0,a.kt)("th",{parentName:"tr",align:null},"Description"))),(0,a.kt)("tbody",{parentName:"table"},(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Base")),(0,a.kt)("td",{parentName:"tr",align:null},"Revolve around the fluidity of robot motion by limiting rapid changes and considering possible collisions between the links of the robot")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Bounding")),(0,a.kt)("td",{parentName:"tr",align:null},"Limit the space within which joints can assume angles and links can move or be oriented")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Matching")),(0,a.kt)("td",{parentName:"tr",align:null},"Specify exact positions and orientations of links or angles of joints, while bounding behavior properties set limits")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Mirroring")),(0,a.kt)("td",{parentName:"tr",align:null},"Allow users to mirror the current state of a link's position or orientation in a different link, or the current angle of one joint in another")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Liveliness")),(0,a.kt)("td",{parentName:"tr",align:null},"Allow adding smooth, coordinated motion to joint angles or link positions/orientations")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Forces")),(0,a.kt)("td",{parentName:"tr",align:null},"Proximating the physcial forces applied to the robot")))),(0,a.kt)("h2",{id:"customizing-the-priority-of-the-objective"},"Customizing the priority of the Objective"),(0,a.kt)("p",null,"The priority of the ",(0,a.kt)("inlineCode",{parentName:"p"},"Objective")," is determined by the ",(0,a.kt)("inlineCode",{parentName:"p"},"weight")," parameter. Higher ",(0,a.kt)("inlineCode",{parentName:"p"},"weight")," indicates a higher priority for an ",(0,a.kt)("inlineCode",{parentName:"p"},"objective"),". A value between 0 and 5 is suggestive, 5 to 40 is true ",(0,a.kt)("a",{parentName:"p",href:"../Goals/goal"},(0,a.kt)("inlineCode",{parentName:"a"},"goal-driven"))," behavior, and after that it behaves more like a constraint."),(0,a.kt)("p",null,(0,a.kt)("inlineCode",{parentName:"p"},"Weight")," can be adjusted in real-time, allowing for prioritization of certain ",(0,a.kt)("inlineCode",{parentName:"p"},"objectives")," and ",(0,a.kt)("a",{parentName:"p",href:"../Goals/goal"},(0,a.kt)("inlineCode",{parentName:"a"},"goals")),", or the deactivation of others, based on the current needs of the developer. Since ",(0,a.kt)("inlineCode",{parentName:"p"},"Objectives")," are organized by key, and atomic updates are possible for ",(0,a.kt)("inlineCode",{parentName:"p"},"goals")," and ",(0,a.kt)("inlineCode",{parentName:"p"},"weights"),", only the needed changes need to be included each round of ",(0,a.kt)("a",{parentName:"p",href:"../Solver/Methods/solve"},(0,a.kt)("inlineCode",{parentName:"a"},"solve")),"."),(0,a.kt)("h2",{id:"import"},"Import"),(0,a.kt)("p",null,"For Javascript, there is no need for additional import. Only a object need to be constructed. "),(0,a.kt)(o.Z,{mdxType:"Tabs"},(0,a.kt)(i.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-js"},'const objectives = {\n  "eePosition": {\n          type: "PositionMatch",\n          name: "EE Position",\n          link: "torso",\n          weight: 2},\n  "eeRotation": {\n          type: "OrientationMatch",\n          name: "EE Rotation",\n          link: "base",\n          weight: 3}}\n'))),(0,a.kt)(i.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-py"},"from lively import PositionMatchObjective \nfrom lively import OrientationMatchObjective\n"))),(0,a.kt)(i.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-rust"},"use lively::objectives::core::matching::PositionMatchObjective;\nuse lively::objectives::core::matching::OrientationMatchObjective;\n")))))}m.isMDXComponent=!0}}]);