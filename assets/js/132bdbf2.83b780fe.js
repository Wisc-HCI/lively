"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[769],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>b});var r=n(7294);function a(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){a(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,r,a=function(e,t){if(null==e)return{};var n,r,a={},o=Object.keys(e);for(r=0;r<o.length;r++)n=o[r],t.indexOf(n)>=0||(a[n]=e[n]);return a}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(r=0;r<o.length;r++)n=o[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(a[n]=e[n])}return a}var s=r.createContext({}),c=function(e){var t=r.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},p=function(e){var t=c(e.components);return r.createElement(s.Provider,{value:t},e.children)},u="mdxType",d={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},m=r.forwardRef((function(e,t){var n=e.components,a=e.mdxType,o=e.originalType,s=e.parentName,p=l(e,["components","mdxType","originalType","parentName"]),u=c(n),m=a,b=u["".concat(s,".").concat(m)]||u[m]||d[m]||o;return n?r.createElement(b,i(i({ref:t},p),{},{components:n})):r.createElement(b,i({ref:t},p))}));function b(e,t){var n=arguments,a=t&&t.mdxType;if("string"==typeof e||a){var o=n.length,i=new Array(o);i[0]=m;var l={};for(var s in t)hasOwnProperty.call(t,s)&&(l[s]=t[s]);l.originalType=e,l[u]="string"==typeof e?e:a,i[1]=l;for(var c=2;c<o;c++)i[c]=n[c];return r.createElement.apply(null,i)}return r.createElement.apply(null,n)}m.displayName="MDXCreateElement"},5162:(e,t,n)=>{n.d(t,{Z:()=>i});var r=n(7294),a=n(6010);const o="tabItem_Ymn6";function i(e){let{children:t,hidden:n,className:i}=e;return r.createElement("div",{role:"tabpanel",className:(0,a.Z)(o,i),hidden:n},t)}},5488:(e,t,n)=>{n.d(t,{Z:()=>m});var r=n(7462),a=n(7294),o=n(6010),i=n(2389),l=n(7392),s=n(7094),c=n(2466);const p="tabList__CuJ",u="tabItem_LNqP";function d(e){const{lazy:t,block:n,defaultValue:i,values:d,groupId:m,className:b}=e,f=a.Children.map(e.children,(e=>{if((0,a.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)})),v=d??f.map((e=>{let{props:{value:t,label:n,attributes:r}}=e;return{value:t,label:n,attributes:r}})),h=(0,l.l)(v,((e,t)=>e.value===t.value));if(h.length>0)throw new Error(`Docusaurus error: Duplicate values "${h.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`);const g=null===i?i:i??f.find((e=>e.props.default))?.props.value??f[0].props.value;if(null!==g&&!v.some((e=>e.value===g)))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${g}" but none of its children has the corresponding value. Available values are: ${v.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);const{tabGroupChoices:k,setTabGroupChoices:y}=(0,s.U)(),[N,O]=(0,a.useState)(g),w=[],{blockElementScrollPositionUntilNextRender:j}=(0,c.o5)();if(null!=m){const e=k[m];null!=e&&e!==N&&v.some((t=>t.value===e))&&O(e)}const P=e=>{const t=e.currentTarget,n=w.indexOf(t),r=v[n].value;r!==N&&(j(t),O(r),null!=m&&y(m,String(r)))},T=e=>{let t=null;switch(e.key){case"Enter":P(e);break;case"ArrowRight":{const n=w.indexOf(e.currentTarget)+1;t=w[n]??w[0];break}case"ArrowLeft":{const n=w.indexOf(e.currentTarget)-1;t=w[n]??w[w.length-1];break}}t?.focus()};return a.createElement("div",{className:(0,o.Z)("tabs-container",p)},a.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.Z)("tabs",{"tabs--block":n},b)},v.map((e=>{let{value:t,label:n,attributes:i}=e;return a.createElement("li",(0,r.Z)({role:"tab",tabIndex:N===t?0:-1,"aria-selected":N===t,key:t,ref:e=>w.push(e),onKeyDown:T,onClick:P},i,{className:(0,o.Z)("tabs__item",u,i?.className,{"tabs__item--active":N===t})}),n??t)}))),t?(0,a.cloneElement)(f.filter((e=>e.props.value===N))[0],{className:"margin-top--md"}):a.createElement("div",{className:"margin-top--md"},f.map(((e,t)=>(0,a.cloneElement)(e,{key:t,hidden:e.props.value!==N})))))}function m(e){const t=(0,i.Z)();return a.createElement(d,(0,r.Z)({key:String(t)},e))}},8090:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>p,contentTitle:()=>s,default:()=>m,frontMatter:()=>l,metadata:()=>c,toc:()=>u});var r=n(7462),a=(n(7294),n(3905)),o=n(5488),i=n(5162);const l={},s="Objective",c={unversionedId:"API/Objectives/index",id:"API/Objectives/index",title:"Objective",description:"Lively allows for a wide range of robot with which users program robot motion. These 24 properties, which serve as building blocks for defining the behavior and motion of the robot, fit into five categories:",source:"@site/docs/API/Objectives/index.mdx",sourceDirName:"API/Objectives",slug:"/API/Objectives/",permalink:"/docs/API/Objectives/",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Collision Settings",permalink:"/docs/API/Info/collisionSettingInfo"},next:{title:"Base",permalink:"/docs/API/Objectives/base"}},p={},u=[{value:"Import",id:"import",level:2}],d={toc:u};function m(e){let{components:t,...n}=e;return(0,a.kt)("wrapper",(0,r.Z)({},d,n,{components:t,mdxType:"MDXLayout"}),(0,a.kt)("h1",{id:"objective"},"Objective"),(0,a.kt)("p",null,"Lively allows for a wide range of robot with which users program robot motion. These 24 properties, which serve as building blocks for defining the behavior and motion of the robot, fit into five categories:\n",(0,a.kt)("inlineCode",{parentName:"p"},"Base"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Bounding"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Matchting"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Mirroring"),", ",(0,a.kt)("inlineCode",{parentName:"p"},"Liveliness"),", and ",(0,a.kt)("inlineCode",{parentName:"p"},"Forces")," listed blow. "),(0,a.kt)("table",null,(0,a.kt)("thead",{parentName:"table"},(0,a.kt)("tr",{parentName:"thead"},(0,a.kt)("th",{parentName:"tr",align:null},"Category"),(0,a.kt)("th",{parentName:"tr",align:null},"Description"))),(0,a.kt)("tbody",{parentName:"table"},(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Base")),(0,a.kt)("td",{parentName:"tr",align:null},"Revolve around the fluidity of robot motion by limiting rapid changes and considering possible collisions between the links of the robot")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Bounding")),(0,a.kt)("td",{parentName:"tr",align:null},"Limit the space within which joints can assume angles and links can move or be oriented")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Matching")),(0,a.kt)("td",{parentName:"tr",align:null},"Specify exact positions and orientations of links or angles of joints, while bounding behavior properties set limits")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Mirroring")),(0,a.kt)("td",{parentName:"tr",align:null},"Allow users to mirror the current state of a link's position or orientation in a different link, or the current angle of one joint in another")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Liveliness")),(0,a.kt)("td",{parentName:"tr",align:null},"Allow adding smooth, coordinated motion to joint angles or link positions/orientations")),(0,a.kt)("tr",{parentName:"tbody"},(0,a.kt)("td",{parentName:"tr",align:null},(0,a.kt)("inlineCode",{parentName:"td"},"Forces")),(0,a.kt)("td",{parentName:"tr",align:null},"Proximating the physcial forces applied to the robot")))),(0,a.kt)("h2",{id:"import"},"Import"),(0,a.kt)("p",null,"For Javascript, there is no need for additional import. Only a object need to be constructed. "),(0,a.kt)(o.Z,{mdxType:"Tabs"},(0,a.kt)(i.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-js"},'"eePosition": {\n          type: "PositionMatch",\n          name: "EE Position",\n          link: attachmentLink,\n          weight: 50},\n"eeRotation": {\n          type: "OrientationMatch",\n          name: "EE Rotation",\n          link: attachmentLink,\n          weight: 25},\n'))),(0,a.kt)(i.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-py"},"from lively import PositionMatchObjective \nfrom lively import OrientationMatchObjective\n"))),(0,a.kt)(i.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-rust"},"use lively::objectives::core::matching::PositionMatchObjective;\nuse lively::objectives::core::matching::OrientationMatchObjective;\n")))))}m.isMDXComponent=!0}}]);