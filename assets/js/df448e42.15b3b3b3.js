"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[3020],{3905:(e,t,n)=>{n.d(t,{Zo:()=>u,kt:()=>h});var a=n(7294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function l(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function o(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?l(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):l(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function i(e,t){if(null==e)return{};var n,a,r=function(e,t){if(null==e)return{};var n,a,r={},l=Object.keys(e);for(a=0;a<l.length;a++)n=l[a],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var l=Object.getOwnPropertySymbols(e);for(a=0;a<l.length;a++)n=l[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var s=a.createContext({}),p=function(e){var t=a.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):o(o({},t),e)),n},u=function(e){var t=p(e.components);return a.createElement(s.Provider,{value:t},e.children)},c="mdxType",d={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},m=a.forwardRef((function(e,t){var n=e.components,r=e.mdxType,l=e.originalType,s=e.parentName,u=i(e,["components","mdxType","originalType","parentName"]),c=p(n),m=r,h=c["".concat(s,".").concat(m)]||c[m]||d[m]||l;return n?a.createElement(h,o(o({ref:t},u),{},{components:n})):a.createElement(h,o({ref:t},u))}));function h(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var l=n.length,o=new Array(l);o[0]=m;var i={};for(var s in t)hasOwnProperty.call(t,s)&&(i[s]=t[s]);i.originalType=e,i[c]="string"==typeof e?e:r,o[1]=i;for(var p=2;p<l;p++)o[p]=n[p];return a.createElement.apply(null,o)}return a.createElement.apply(null,n)}m.displayName="MDXCreateElement"},5162:(e,t,n)=>{n.d(t,{Z:()=>o});var a=n(7294),r=n(6010);const l="tabItem_Ymn6";function o(e){let{children:t,hidden:n,className:o}=e;return a.createElement("div",{role:"tabpanel",className:(0,r.Z)(l,o),hidden:n},t)}},5488:(e,t,n)=>{n.d(t,{Z:()=>m});var a=n(7462),r=n(7294),l=n(6010),o=n(2389),i=n(7392),s=n(7094),p=n(2466);const u="tabList__CuJ",c="tabItem_LNqP";function d(e){const{lazy:t,block:n,defaultValue:o,values:d,groupId:m,className:h}=e,y=r.Children.map(e.children,(e=>{if((0,r.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)})),f=d??y.map((e=>{let{props:{value:t,label:n,attributes:a}}=e;return{value:t,label:n,attributes:a}})),b=(0,i.l)(f,((e,t)=>e.value===t.value));if(b.length>0)throw new Error(`Docusaurus error: Duplicate values "${b.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`);const g=null===o?o:o??y.find((e=>e.props.default))?.props.value??y[0].props.value;if(null!==g&&!f.some((e=>e.value===g)))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${g}" but none of its children has the corresponding value. Available values are: ${f.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);const{tabGroupChoices:k,setTabGroupChoices:N}=(0,s.U)(),[v,A]=(0,r.useState)(g),C=[],{blockElementScrollPositionUntilNextRender:w}=(0,p.o5)();if(null!=m){const e=k[m];null!=e&&e!==v&&f.some((t=>t.value===e))&&A(e)}const O=e=>{const t=e.currentTarget,n=C.indexOf(t),a=f[n].value;a!==v&&(w(t),A(a),null!=m&&N(m,String(a)))},T=e=>{let t=null;switch(e.key){case"Enter":O(e);break;case"ArrowRight":{const n=C.indexOf(e.currentTarget)+1;t=C[n]??C[0];break}case"ArrowLeft":{const n=C.indexOf(e.currentTarget)-1;t=C[n]??C[C.length-1];break}}t?.focus()};return r.createElement("div",{className:(0,l.Z)("tabs-container",u)},r.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,l.Z)("tabs",{"tabs--block":n},h)},f.map((e=>{let{value:t,label:n,attributes:o}=e;return r.createElement("li",(0,a.Z)({role:"tab",tabIndex:v===t?0:-1,"aria-selected":v===t,key:t,ref:e=>C.push(e),onKeyDown:T,onClick:O},o,{className:(0,l.Z)("tabs__item",c,o?.className,{"tabs__item--active":v===t})}),n??t)}))),t?(0,r.cloneElement)(y.filter((e=>e.props.value===v))[0],{className:"margin-top--md"}):r.createElement("div",{className:"margin-top--md"},y.map(((e,t)=>(0,r.cloneElement)(e,{key:t,hidden:e.props.value!==v})))))}function m(e){const t=(0,o.Z)();return r.createElement(d,(0,a.Z)({key:String(t)},e))}},6041:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>u,contentTitle:()=>s,default:()=>m,frontMatter:()=>i,metadata:()=>p,toc:()=>c});var a=n(7462),r=(n(7294),n(3905)),l=n(5488),o=n(5162);const i={},s="Cylinder",p={unversionedId:"API/Shapes/cylinder",id:"API/Shapes/cylinder",title:"Cylinder",description:"A 3D cylinder shape that captures colliders with curvilinear geometric properties.",source:"@site/docs/API/Shapes/cylinder.mdx",sourceDirName:"API/Shapes",slug:"/API/Shapes/cylinder",permalink:"/lively/docs/API/Shapes/cylinder",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Sphere",permalink:"/lively/docs/API/Shapes/sphere"},next:{title:"Capsule",permalink:"/lively/docs/API/Shapes/capsule"}},u={},c=[],d={toc:c};function m(e){let{components:t,...i}=e;return(0,r.kt)("wrapper",(0,a.Z)({},d,i,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"cylinder"},"Cylinder"),(0,r.kt)("p",null,"A 3D cylinder shape that captures colliders with curvilinear geometric properties."),(0,r.kt)("p",null,(0,r.kt)("img",{alt:"img alt",src:n(3086).Z,width:"150",height:"150"})),(0,r.kt)("table",null,(0,r.kt)("thead",{parentName:"table"},(0,r.kt)("tr",{parentName:"thead"},(0,r.kt)("th",{parentName:"tr",align:null},"Parameter"),(0,r.kt)("th",{parentName:"tr",align:null},"Type"),(0,r.kt)("th",{parentName:"tr",align:null},"Description"))),(0,r.kt)("tbody",{parentName:"table"},(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"type")),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"The ",(0,r.kt)("inlineCode",{parentName:"td"},"Cylinder")," shape class")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"name")),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"A user-defined name of the shape")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"frame")),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"The coordinateshe frame this shape belongs to. Can either be ",(0,r.kt)("inlineCode",{parentName:"td"},"world")," or ",(0,r.kt)("inlineCode",{parentName:"td"},"robot frame"))),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"physical")),(0,r.kt)("td",{parentName:"tr",align:null},"boolean"),(0,r.kt)("td",{parentName:"tr",align:null},"True if the collision with this shape is phyiscal, else false")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"length")),(0,r.kt)("td",{parentName:"tr",align:null},"float"),(0,r.kt)("td",{parentName:"tr",align:null},"The length of height of the cylinder")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"radius")),(0,r.kt)("td",{parentName:"tr",align:null},"float"),(0,r.kt)("td",{parentName:"tr",align:null},"The length of ",(0,r.kt)("inlineCode",{parentName:"td"},"radius")," of the cylinder")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"localTransform")),(0,r.kt)("td",{parentName:"tr",align:null},"isometry consisted of translation and rotation"),(0,r.kt)("td",{parentName:"tr",align:null},"Defines the position and rotation of the shape relevant to the specific ",(0,r.kt)("inlineCode",{parentName:"td"},"frame"))))),(0,r.kt)(l.Z,{mdxType:"Tabs"},(0,r.kt)(o.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-js"},"let shape = {\n  type:'Cylinder',\n  name:'zone',\n  frame: 'world',\n  physical: false,\n  length:0.2,\n  radius:0.1,\n  localTransform: {translation:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0]} // [x, y, z, w] ordering for quaternion\n  }\n"))),(0,r.kt)(o.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},"shape = CylinderShape(\n  name=\"zone\",\n  frame='world',\n  physical=False,\n  length=0.2,\n  radius=0.1,\n  local_transform=Transform.identity())\n"))),(0,r.kt)(o.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"},'let transform = Isometry3::from_parts(\n         Translation3::new(\n             0.0,\n             0.0,\n             0.0,\n         ),\n         UnitQuaternion::from_quaternion(Quaternion::new(\n             0.0,\n             0.0,\n             -0.7069999677447771,\n             0.7072135784958345,\n         )),\n     );\nlet cylinder_1 =  Shape::Cylinder(CylinderShape::new(\n             "cylinder".to_string(),\n             "world".to_string(),\n             true,\n             0.2,\n             0.1,\n             transform,\n         ));\n')))))}m.isMDXComponent=!0},3086:(e,t,n)=>{n.d(t,{Z:()=>a});const a="data:image/gif;base64,R0lGODlhlgCWAKIAAP///8zMzJmZmWZmZjMzMwAAAAAAAAAAACH5BAQUAP8ALAAAAACWAJYAAAP/CLrc/jDKSau9OOvNu/9gKI5kaZ5oqq5s675wLM90bd94ru987//AoHBILBqPyKRyyWw6n9CodEqt0gLYgWAwIBC6hbB4LP56tVoB1lrCbsHlcxobANQddfpWYPaWuWp3bBUBWwRhcgJ2LIVcfgVfaoMMjYeQA2s6lYhaglGGl4pGoJFNhYdfnksBYASSRacFWmx8Xq8/AV6ukwuxszwCl0iZHqyHojaFkMhAAWJqiCPOXzW1qkHBspYFJVsFzC25u0cDYnXj3V7XKOXgRdkELe0rxutE5d8u4ipeTdkDL+qdoNYEn7sVXUwQ9BcGYIx+I/hAMTgjn4gC9o44MyeD/09GDBJ5uUDnwaJIFuVCYDwZjhsIl80QfczhTOVMG2SG1Hzp8AeZn0CDCh1KNGjKl/GA5BRySOVKXDKFZFMZjKWKbzBLAlholQS1rB1ccu0KAiJYDjBl3SQ7YZ+CsxuyqmPLwe1blQ2C9aRrYR4DuBrOijvIN+/cBoAzAA5GsvACPiYR443AmHBXxnsdJMawWQHmtVRYLaPQ2UJpz+VcgTYF+RfpyRZO2VptJJZqzrAx2HbNhNQt3C8/kOoEa7hl07mLgQpFm97ySM0jnH5tYhMi6C0EGNqG3cT0Cd87uHFUBtIXQIHyULKj502XRw1dBWIRXvqVLG90bRuqHxAmYv8x1AeBgI5R9wGBBUoAk3bEAZfgRX+F0RyCDz4gl4SKVQjhApZMqGFuHbKH3IcHMmAJZJBUQCGJd3GIiDeyGMhiXCaGochGSSk4Y1g1roSjjDs6uBWGP4IXJI0u+hiNkUdmmGQeS+rYpJAhFinllCMqUGWU9mGZZTaQbGQjk14CiVaZKiaHpoVqrilZcG52WWKcA7ZJJwArLpAniXvieSebcP75l5109tmnhoYK+uacirbIaKOJNuqoVpL6GSikhMYZqaSbYnqpop2CmqmboQpa6p+n3plqoaOuuaqmraL56po7HRjdjrV60NidXIQQkqK7onUriVWJUCyqx2mQyp2YEHmVWZnNlhAtlnYNRMCwbFXLToxNlvMsCoPtaE0MlX0IWbIqlFtgLd92dAgm2ab2Gw7e3MaLL+jWYNu8UOyLbQy75StVK/weYdwiOh08hXXd0fRcg/cuZx7E6W73R8FdjQcHJBOjRwfCCuTRHhfv7deFdgDiip8juhRl3nlofFzpzDTXbPPNOOes88489+zzz0AHLfSUCQAAOw=="}}]);