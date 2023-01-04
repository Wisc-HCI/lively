"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[1631],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>g});var a=n(7294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function l(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?l(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):l(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function o(e,t){if(null==e)return{};var n,a,r=function(e,t){if(null==e)return{};var n,a,r={},l=Object.keys(e);for(a=0;a<l.length;a++)n=l[a],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var l=Object.getOwnPropertySymbols(e);for(a=0;a<l.length;a++)n=l[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var s=a.createContext({}),u=function(e){var t=a.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},p=function(e){var t=u(e.components);return a.createElement(s.Provider,{value:t},e.children)},d="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},c=a.forwardRef((function(e,t){var n=e.components,r=e.mdxType,l=e.originalType,s=e.parentName,p=o(e,["components","mdxType","originalType","parentName"]),d=u(n),c=r,g=d["".concat(s,".").concat(c)]||d[c]||m[c]||l;return n?a.createElement(g,i(i({ref:t},p),{},{components:n})):a.createElement(g,i({ref:t},p))}));function g(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var l=n.length,i=new Array(l);i[0]=c;var o={};for(var s in t)hasOwnProperty.call(t,s)&&(o[s]=t[s]);o.originalType=e,o[d]="string"==typeof e?e:r,i[1]=o;for(var u=2;u<l;u++)i[u]=n[u];return a.createElement.apply(null,i)}return a.createElement.apply(null,n)}c.displayName="MDXCreateElement"},5162:(e,t,n)=>{n.d(t,{Z:()=>i});var a=n(7294),r=n(6010);const l="tabItem_Ymn6";function i(e){let{children:t,hidden:n,className:i}=e;return a.createElement("div",{role:"tabpanel",className:(0,r.Z)(l,i),hidden:n},t)}},5488:(e,t,n)=>{n.d(t,{Z:()=>c});var a=n(7462),r=n(7294),l=n(6010),i=n(2389),o=n(7392),s=n(7094),u=n(2466);const p="tabList__CuJ",d="tabItem_LNqP";function m(e){const{lazy:t,block:n,defaultValue:i,values:m,groupId:c,className:g}=e,h=r.Children.map(e.children,(e=>{if((0,r.isValidElement)(e)&&"value"in e.props)return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)})),f=m??h.map((e=>{let{props:{value:t,label:n,attributes:a}}=e;return{value:t,label:n,attributes:a}})),b=(0,o.l)(f,((e,t)=>e.value===t.value));if(b.length>0)throw new Error(`Docusaurus error: Duplicate values "${b.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`);const k=null===i?i:i??h.find((e=>e.props.default))?.props.value??h[0].props.value;if(null!==k&&!f.some((e=>e.value===k)))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${k}" but none of its children has the corresponding value. Available values are: ${f.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);const{tabGroupChoices:v,setTabGroupChoices:y}=(0,s.U)(),[w,O]=(0,r.useState)(k),N=[],{blockElementScrollPositionUntilNextRender:C}=(0,u.o5)();if(null!=c){const e=v[c];null!=e&&e!==w&&f.some((t=>t.value===e))&&O(e)}const E=e=>{const t=e.currentTarget,n=N.indexOf(t),a=f[n].value;a!==w&&(C(t),O(a),null!=c&&y(c,String(a)))},B=e=>{let t=null;switch(e.key){case"Enter":E(e);break;case"ArrowRight":{const n=N.indexOf(e.currentTarget)+1;t=N[n]??N[0];break}case"ArrowLeft":{const n=N.indexOf(e.currentTarget)-1;t=N[n]??N[N.length-1];break}}t?.focus()};return r.createElement("div",{className:(0,l.Z)("tabs-container",p)},r.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,l.Z)("tabs",{"tabs--block":n},g)},f.map((e=>{let{value:t,label:n,attributes:i}=e;return r.createElement("li",(0,a.Z)({role:"tab",tabIndex:w===t?0:-1,"aria-selected":w===t,key:t,ref:e=>N.push(e),onKeyDown:B,onClick:E},i,{className:(0,l.Z)("tabs__item",d,i?.className,{"tabs__item--active":w===t})}),n??t)}))),t?(0,r.cloneElement)(h.filter((e=>e.props.value===w))[0],{className:"margin-top--md"}):r.createElement("div",{className:"margin-top--md"},h.map(((e,t)=>(0,r.cloneElement)(e,{key:t,hidden:e.props.value!==w})))))}function c(e){const t=(0,i.Z)();return r.createElement(m,(0,a.Z)({key:String(t)},e))}},6251:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>p,contentTitle:()=>s,default:()=>c,frontMatter:()=>o,metadata:()=>u,toc:()=>d});var a=n(7462),r=(n(7294),n(3905)),l=n(5488),i=n(5162);const o={},s="Capsule",u={unversionedId:"API/Shapes/capsule",id:"API/Shapes/capsule",title:"Capsule",description:"A 3D capsule shape that captures colliders with curvilinear geometric properties.",source:"@site/docs/API/Shapes/capsule.mdx",sourceDirName:"API/Shapes",slug:"/API/Shapes/capsule",permalink:"/docs/API/Shapes/capsule",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Cylinder",permalink:"/docs/API/Shapes/cylinder"},next:{title:"State",permalink:"/docs/API/state"}},p={},d=[],m={toc:d};function c(e){let{components:t,...o}=e;return(0,r.kt)("wrapper",(0,a.Z)({},m,o,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"capsule"},"Capsule"),(0,r.kt)("p",null,"A 3D capsule shape that captures colliders with curvilinear geometric properties."),(0,r.kt)("p",null,(0,r.kt)("img",{alt:"img alt",src:n(8482).Z,width:"122",height:"203"})),(0,r.kt)("table",null,(0,r.kt)("thead",{parentName:"table"},(0,r.kt)("tr",{parentName:"thead"},(0,r.kt)("th",{parentName:"tr",align:null},"Parameter"),(0,r.kt)("th",{parentName:"tr",align:null},"Type"),(0,r.kt)("th",{parentName:"tr",align:null},"Description"))),(0,r.kt)("tbody",{parentName:"table"},(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"type")),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"The ",(0,r.kt)("inlineCode",{parentName:"td"},"Capsule")," shape class")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"name")),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"A user-defined name of the shape")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"frame")),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"The coordinateshe frame this shape belongs to. Can either be ",(0,r.kt)("inlineCode",{parentName:"td"},"world")," or ",(0,r.kt)("inlineCode",{parentName:"td"},"robot frame"))),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"physical")),(0,r.kt)("td",{parentName:"tr",align:null},"boolean"),(0,r.kt)("td",{parentName:"tr",align:null},"True if the collision with this shape is phyiscal, else false")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"length")),(0,r.kt)("td",{parentName:"tr",align:null},"float"),(0,r.kt)("td",{parentName:"tr",align:null},"The length of height of the capsule")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"radius")),(0,r.kt)("td",{parentName:"tr",align:null},"float"),(0,r.kt)("td",{parentName:"tr",align:null},"The length of ",(0,r.kt)("inlineCode",{parentName:"td"},"radius")," of the capsule")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"localTransform")),(0,r.kt)("td",{parentName:"tr",align:null},"isometry consisted of translation and rotation"),(0,r.kt)("td",{parentName:"tr",align:null},"Defines the position and rotation of the shape relevant to the specific ",(0,r.kt)("inlineCode",{parentName:"td"},"frame"))))),(0,r.kt)(l.Z,{mdxType:"Tabs"},(0,r.kt)(i.Z,{value:"js",label:"Javascript",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-js"},"let shape = {\n  type:'Capsule',\n  name:'pill',\n  frame: 'world',\n  physical: true,\n  length:0.2,\n  radius:0.1,\n  localTransform: {translation:[0.0,0.0,0.0],rotation:[1.0,0.0,0.0,0.0]} // [x, y, z, w] ordering for quaternion\n  }\n"))),(0,r.kt)(i.Z,{value:"py",label:"Python",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-py"},"shape = CapsuleShape(\n  name=\"pill\",\n  frame='world',\n  physical=True,\n  length=0.2,\n  radius=0.1,\n  local_transform=Transform.identity())\n"))),(0,r.kt)(i.Z,{value:"rust",label:"Rust",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-rust"},'let transform = Isometry3::from_parts(\n         Translation3::new(\n             0.0,\n             0.0,\n             0.0,\n         ),\n         UnitQuaternion::from_quaternion(Quaternion::new(\n             0.0,\n             0.0,\n             -0.7069999677447771,\n             0.7072135784958345,\n         )),\n     );\nlet capsule_1 =  Shape::Capsule(CapsuleShape::new(\n             "capsule".to_string(),\n             "world".to_string(),\n             true,\n             0.5,\n             0.25,\n             transform,\n         ));\n')))))}c.isMDXComponent=!0},8482:(e,t,n)=>{n.d(t,{Z:()=>a});const a="data:image/jpeg;base64,iVBORw0KGgoAAAANSUhEUgAAAHoAAADLCAIAAAAurRBjAAAeJUlEQVR42u2dy29c133Hh5QoStSDoihLsgxLMOwUSppCiWGjTdECQbZFCrTopot4FfQf6K6Ltpt6F7dokboo0qbLbgI0jYGmQFCkLRCgXTiOC8cSbepBvSlRJEVySM7r9sv5cL766Y5mhkPNkEPiHBAXd+7jPL7nd37vc1nIelQqlUqpVNLJwsJCvFgsFnWytram86WlJa6X6qVcLut8bm5OR50vLy/rpFqtUg9HvahblXpxtXpyfX19dXWVi7VaTc+ocp3o5+LiYjaopdDDujRggyKUBQqQXb9+3Q/Mzs4KFOZARZDpqIt6UdcfP34suAWlHpifn+d1psGTpLutOqB3Hz586Knaz3ALuGq9CDiRpK8/evSIo4kO8uf67du3PT1AH9GknqmpKVUr+tWR68J0ZWVFmDK7TFKcP1bMPqdu6JeRCzXBKjg0fuHC8tcJcMRy79493arVy4MHD/S6JubWrVvwIgBlbvSYJgCOEedDi0OTodVA5QNL2r2EWwS1Ui+5izpy0SiIGAXQRtv1cv78+ZGRkRMnTuh44MCBw4cPDw8PHz9+XFd4gHdF/gBNbev1AgPJ8RMq3+dwM3hBeefOHZ3cuHEDXASTLhrc06dPC9OXXnoJHBGSIue7d++yOKBQvSJwIW09OTQ0dPDgQc2Hjn4R9O/fv6/Hrly5Ao968uSJb+1zZqKRx59CUACNjo4eO3bMGKGcQPWgqXliYsRJQArZiDC4efOmzsUokKVIUVWrFWDa112mDWFrJrOf4QYjYSH6EigQspiAINNFOAnnLH9BEzU2IY48BFargEZTF/1i5CGCHtB5WHeRFnsMbqCBGIFSiIh2LOuEKc/wACdQ3NjYmFDL8fEtllqXxQznzJkz4O4eMoUejtfTIMKNMqBeispEhlEiCWgoDkA5n5ycZLSCADaKZtJvuOmk6ACGg+zVT/pvXV70PgjaYUu4WY9YDUAsEgZZyAd7RJOBCsHz09PTEJGu24bsK9yRKUET0iwlUaXhaL59i+7tOuLteLc1WfXbwgfKFdBCUxMgoGGatt2vXbvG8GwW9rWgighWdUZA0w3NgZoWT5NCKTIXyrb4BxduwNJihEYEMSuUc+kGEoY6l0nCZGjAGpiNvchG+1dYQ9EhY2GDI+XixYtMgFgfczOIcBspFqDWqRVqRBO8W8ofGrflKta2LsrcaOPf6GFRcyiRdNv6KKqkeqXeStmne21kwG7CneO8WNWCEgsFEQqsrABIDHOch3emiHugFFnXhETUVa7QQ/0UN9cEDCjcdgYxElQuAa0jvEVd10g0DK1QLEPElG3F6Inta1GXWILRZ2BVih5ujrZQGFC4LeXsRxanRjPZ00VKC6vBWnm5XnYZbmgW606IS63++OOPs71fYOUwQElUG0q7z0zUG82/OnTkyBHoHR69D4p9OBqUGKY1rt2kbsIxUl3xKe8PoCVyYOVCXFrWDivjhY5UAKdDL9wBPbrfBdcumuvo6Kg4+E5aQIX2WNMzkbkoYmfEd781B6oSxJCR9HHJ/52xD9rBPTMzc+rUKZQqXMlQxF6HmxgeppCqHRoaykL0eTep2w6TaF7udbitkmPHiZ4OHTq0Y+GIAuaJ3cFkhkg82lufNeIsttDURchByxBnELzP2qvNCk0VYUZYE6GvkZGRo0ePDndZxsbG9O6xY8eoBNXCEcusEcFRT+iGGsUKs+fdJKwnRUn2rFGtO09ehtNmek/dZhGoH1iPJmdoXJ3AYlRfWYmff/65DfeskUxjjUo1CCOxo4P1AkAMeGpqignuqvAKoBMq0lF2uZTUiYmJXHAux45jfEf9d6KEE2OwlnEHWUHUlZ4zmQLhc+pF50fzA2vIUzMfMYWQMeIZHk59HcGXweuWWb+zISQStmfCYdBGinMknulnGsbHx8WOz507BykQ6jRpYzcQAHFVOtfy1YKjfvrsGF5feDc4Yq/bhc1Pt6pnsDAdzRHXUy8B98qVKxEXFTyI9J6VYSaDAd0tdet1wYFrm0pYeTkjRQ9oAojhCXeGA6vRkUSi6EhhsDHKbHHVcwWxIOwgZJaPaBN8ZWs5Rq7xmBZYwmfPnmUBehisWQfRiZyZ/K3boPNso6P2A/sKKKtpdZI4shC00CPNCuCkX2vJmuHoRfWNh6EDps2Kr/lqLomlN9QdiU5Nqm17X8XWPYCYZONF6nClLSCmzUkmel3MGpEloF9EwxUrIPHBmZ6aXcS4KNqrSi2aOHAUq3X6Q8qKRgdF8xipXvalaGXYV9F73g3jgxFLuOmExkgz0wlZInQlko/fYlQ2Ox2AJ+9Jg7Ggi4y40mWJWRJ4coyv3etOIUIngUIdPo0LVPQuBUkPk1AXHSnWWPphZBSiAPRy0xWTgx8lkRWI0besBpiuncNn9kd+iJOJYf3b6yvOd3KXs0YmG9Os+pv5LHyGZWHZ7tbVGS9WFXVMz4uXOhzRD7OuQICD9GoRsmPtEoORO9se2133/IubS3BO3BIaNdqUFW2RfDQ1OorKOH8sZeZJ9Or8dEDL6920ndWTgNWq1SZeYwXAH/Y03NFqo9y9e1e8hZHaeoKJtxeVTpbzK3BgcCN6FwXJBtxqjJAjTaorly5dykKqo3VEKxh7Gm4Wq5gMoIuzWy0RkR2uF4OoFdBKlmSNNHPVSRA8lxtCDbn1UcgaWTgHDhxAlVYtzWEEPdNxtveKA9YUZypBtEhTEL7iqMKLW20SJSzwYlK56lH9bL2I/gy7mwq+IQ0pCkbrpwSZsmf3Fez1AtkCk85FRpBkBCHa9M1Fr+sV9DrebQ4GoIDGAG+BZaVHrXU6US2G0mM64J4uqOGs/RxArHIb1dBvG900mibefaHJYz+G4xjR6H2a0y6eZZoXrJFvwJ6ywd6HsfUiHO23QUVZqxdUBuEggrPy2j78Ek2QWKev4zMwBRfsexobG+MF7+2IG+5yxL53C47ZrBH4znnAhZp9vGQwtyp4YyTw+Hn06FGfu0j7yPHnghU+qZ+QMEsM6NUnUr+wdPcH47YiaE+Dnbcao4AjFyxrbOjiGT8muOy2Rde2idtRvG0a8aqLVDTVriNcZWZmBsHq7OEdCJ71u5DrLVwYuJY5lARLESdBIYZx26mCJWhFRZVoBRw/fhx1zlsXO5pFBbsdcPVFkRIXIBr3PlAEYwiQBHAw4jraN6qLn4yu2qf2YaEgpVlwOwizFXA2pDC7GV977TV7BNkHpn4QsrHLdB+k9ViOmaWIKqEt+KyOdqmLuyLD7HrTkwJHPy9evAgH1xU9rCe3sjmmYI6sl2E9RHPY5mU2orq256cewGI/eNzyjeNTLIKwDtetrhAhEazGFGeLqNuyNya2t4SbJaAZHh8fh5lA46wRWJhFwQAqgt3uLBFk9ncaa49XJxMTE9gfRlavRCVddj9CUjNE8IvJ2MrmmI1wLQuHgDScWuifPHmSiphhNMcBFJXb2Dr1jCLccOXDvi9cuOCf+Agji8CK0cmJEyeyRjKtTP+cQd9BVKpeEe+RI0e0NDThdlHiRrAPdjD17m7hxmVqCrPSjSLgXZr+zgfFOzEwSs3lMYuowVPYObwgTKNOzs4ERxhMAk70QSv3ZBI0yOX95JRQBA6vbG+PjNNF0MwIGTvKYW0qujC9Td/Ksl5kqxwROHrizbWqTTQXHZwQmcO5PKxZ0VGGIQq0lbctwY1R71nF7+7Pu8TUEXEbLy5/DyNyGOuz1pzYOKxK/Nj2RC6IxCl0xM6+PfXNe+vdkPoQhyA0c1lKdLgV3FkjygyDFgmbKHMn6mFHBlCwae93THrWmdQJKeZ0zglNYj6MliwGoqveqIE7DU+m62FrWtbYO9NV8cTHeCnXr169muOb3q0jolZXJYewsD1Pd+7cYQ+RV0wbuKempiJJaeCED6V388EamftblGoFf7iCyBlDcqyaTZIm2Js3b8bghZ0GmgOr5F4WkczVCp54NqhtI7NnqV5yTrjodxZkbBGCeNWrV155hYxLr4PmzbU4p2ykNMPt1QwyIiNJSJLwNWo6Qyuirc5eLaLaXhFq2/E3rnt3k0WBzSfLVZ1IjzSXR6GEBrW6c51wzKmr4tRnWkfJdQaEM9nEVUXLQsT81KwDZMm68qrP7ThuhpsAGMN0Hn6OmYjUJicnt2TmeK+qXoZCwVd9giKQMARPGblaVdX4s7wrnWEjRUUI5I9hCwgCdszTP93dhreLzBYSRUjqRJaYS9iAABeomJ/qlQnIUOoZd8MO52a47T4kKSx6+OInWWKQsx3cYJfVE8ut+kRO5Gy05tmzU4bn1aGoMsKmPX53UdMgrjfUZXHeJSqTs0dpmnw2WvH3rfjpHBVc/lBMNP+8xf+5cPvjVjBALR3V5vAmi0w1i6q2En7ZTGSBus21SSP2x+YiaoDuvaoaLYMxOybDDdwdwbPDIW5+7arwJbucJ4hkT6txTiGiFYx1J1dGPdqsKXt2H+ZzRWUWkhKgJCCWQsm0UcOWrMr2zfTbunvx+tukcHbrmO0WBwjU6ny2hZzCBHeCO8E9sKVX05ngTnDv35LgTnAnuBPcCe4Ed4I7wZ3gTnAnuBPcCe4Ed4I7wZ3gTnAnuBPcCe4Ed4I7wZ3gTnAnuBPcCe4Ed4I7wZ3gTnAnuBPcCe4E956D2/81gU9cZc/u+khw94W6vdfNO24T3H2B21vIKGyr9U6aBHfvqZvvzcZvjCZm0kfeTVXs6mQn9dONdwnu3sLtTzg+d3ddgrv3zERs5J133nn//ff56lQWvk2S4O4x3JXyeq1a/t1v/s4HH/yrrswvLujGeqmiYy3B/eJwx2/vVyulrLJ269qVX3/r8s8//N/1cunh/IKUkvliRZxlPcHda7jXsvLKtU9//tblL37/H/9+ZPTQ8VMv/fGf/PnsUkm64WqCu+dwrz559L3vfufY6NDv/943VcsH//6TX3v7N0TXq7WNvwR3z5lJ8a/f+4tLr786++DOvQf3/+O/f/arX3nr7txSsZrpL8HdY7gf3r3xtbe//OMf/WB9bblUKf/zD/7lD/7wW/cXEtz9gbtcXPjSFy7+z8/+c7X45PHC/Ntf+833/uZvHy4VE9x9gXvm+tQrZ04uzN3PaiVpga994Vfe/973Zciv1zb+Etw9hvuf/uHv3vrql0vrKyvFxenp6ctf/cr0zZmVUrVUTXD33swp16oyc9Z0rNbWN6yeWrVSzUq1rFrLykkR7CPc1bL+gFt/1TriCe4ew71B1LVNrBt/1frn7PRMgrvXPpNNxBPcO+SiqomBRKzFXjb+4CYJ7gT3voS7VtFfgjvBvZfh7vBKgjvBneBOcCe4E9wJ7gR3gjvBneBOcCe4E9wJ7gR3gjvBneBOcCe4E9wJ7gR3gjvBneBOcCe4E9wJ7gR3gjvBneBOcCe4E9wJ7gR3gjvBneBOcO8a3OvrqnuzE48ePap3Tj+rtVql6a/z/wjW6/4s39LS0g7ArSvx/xcvLy9HuMGa7yCpY4uLi7tP3bWqurI6NzffmIDVhYXH24Bbd8v1ouHdu3dPQy0Wi/2G27PrtoS4YJ2fnz979iwPqPC9wM4LYgeoO5yv+4t6Ai38PS2VFiVHWXwwzt946iszuXHjho5zc3N8JtDPHzp0yJ9QW1tbU/f8czepe3m5uLq6sdxmZmZo4vHjx8103ZG6Z2dnhbLeBff4dcS+wi3ijZ9N8114d9b6u2q7ALeg0TKEPLUATd3d/n93aojULX7SUTS9ONwQrHouWjHWiA3gfvjwIXxGXeq42noGdyv4DJO6BT+BG1a6LBKzeuvzzz+HtHvItTtSt1pkFGqUj4zqAY1iZGRE55p1sL5169bOUXcruKvVcrG4PDf3sFRasy6xtLSyjSbETFi56i2TFz8e3D+43W1mGnqXtDx8+LDVkps3b+6oItiSuKtrtUxiTepURatveHj40KHDw8MHCwX/Ddf/OhR1L+q5VtH6DbcXKK2LwMVYuKUn/flonjQ3bwm3dQY/qoaZT9erNqxRsoo1z3BhKC4KEHpGGR4uHBodOjI2XBgq6E911xdmbX2tuvRkLat/ynDjZ9Be2sOhOqUPxGkYGhoaHx9354v1Yi1IRTqD3oUGdWJWBhOg2/4wrk5U1cZu1IZC7ecFLjXD1lREOswQ8ikKz5ZwI9+l35w6dYo2QNDfjY3UxASwkCUWNBLPaiwRpeWVudlHWmh6UkpeqVhcq1YygNZfuZQtL60Z7qjq5QpNwzoppgC+JUzTR48eZQ6snzEK4Gawd+7ccQ3o8joKaI1UDxw7doyTKKJ1V4C4WiGumtWQJoOpUg2igy0xE+hUzWgwnKs62lOl0jdz7+jigwcPRkdH9UoTvpk1ENVQKhermRBcXltfXF6Zf/JkQXTw6OGCDJ9KOSuulAV3VkMBQGlt112mWR0WvhqkSESrUCer9WJWDrLqlXhr7B7Qmx37w8OeNiZbz7MgVDRMrwNuse6BGOrmq8YaLy92gJsn1JK/rGQSE1EwDI3z9u3bbnJyclJA57QivQUJIKD1br1P5UptqVQR9RXX1pc20awT9SYnKWerxUq55Ilsp2+ICMA3dx0S0XWdXL9+HfqtNop+HjlyBKpngELQlUBMOkLF09PTb775ZpwDMxnNovgJxKSpArGcTd9ZM/EZcjbKIlbuL3/5Sx3hj65RTcJzaA+C0ouM3JO88Zm3J1q5S6LxSnVldvb+L37xf7/9W9/4q798v1A48md/+u7yUlmgl9ZrDd3jfquOMn4PD+jVB50IC+gx8hYRCt3QXT3G68hqHdV/s+BPP/2UbkPF0XjRi2pFo7PAPH78uKqirZMnTyLD1JA4iU46akoFsWDzPh1Fm/F/BdAAtxiqbqmjPIDmbxKDuhlkw5zRcfnW3U+yDZaytrg4/8Mf/qhQGHnvO9+t4aqqZY/nlh7cn2uw72pHF5Urx6agOQ0BzsBPzG51mPFbjNuLdPr0aRxMzWY3181jPXAtcQ1QaFhH0KKxPGAKO/Pu3FowaprVSM70Es9AjhXqSbGO+vd/alEibXgCa+v3HnxWrs2VKvM3Z6aE5re//Udf+uLlhflVwb3weB3q1t/Kymq9iWobRVOtYE8Dur8Db94q0OmGmQbUjah0zy0q0XO8jlHPNBPMKyBAXh6yOmBMdGJK3aoRDyPT0xJ96ECW8rSKVInEpX6DtWBFshsR6zPwzXJltVRZ3CDwO8Jaw17++te/8eN/+8naarW4UgVo8e7VYrmxflfakzaj0hFFhfGDqa7QK48c5gbTi8oG/FqdZGIYKZ1/44039BMxYJHAsqAe0QQSEr0bQ/fVV19Fu+sMN5CpJU+1Trjy3NKlZVHdoPLKqowd+PKFCxd++tP/2sUwQivrV6MWvpao0ezwauDfs1jkHjx4kFlhhqzCgVJ0x9vlUDDVSBii2EEgrbpVqpdmf2mr4WnJ2zP12WefXbp0SZTikQwO3Cgw0hfEnWE75o0Wv5ELIdU08Cha8X2ba0mPzAnPAtqexv/yyy/HiS23KG1W+nOLX5Fke/fddy9fvszybOPX3hW4hSwmpUCPdIp+AtPgdalq0kl4WB0+f/68MLTy4xethkY5t0nd2EiSEjpvT3rIEDOWrUS8NBL8Sm4bh85AUTcrVQIM1gxd55x8FsLYOwJd/HpiYkK0D6FonuyDxVLRwKPzshBVGVpt77ZvZiNxGluZgviCGYOX5EDB7VHDiC1R0YjQFwWu2QuaBcaKho/WFHUKqJbXn/JuOxUlZ6Xx2IhvtdhbkXab521e62GR9nq9DBozwUiJep45QFS9UIUxKSkiI7rNhCHemJVmPlGIzjaabE99zyXtNtStZ9Q5KbZoOxBOGxmwi6ISKMbGxkR8ws5+RPwTYMfwsWNPnDgRvYliL6rHzITndYw8fdMjKGpXM9Jszp071+zkbC5DjTLcKK2elOTRUT3TUQuQmu08ai67BTcCDPtD41I/7YAbGRlhFDAQAcVF7nJRvAElksGqhue6xgr2BvC+oxUqU1NT1m+QHj2MDe5AweVt6hNJQXqMwrzLTmbdxbhnvB3D6tsoGx5BrQIJAc3n66+//sknn9ASNpJ6rN7gAGrjjB7AEgW+2GPOf601DqNAiGHxY+xoiQP9VkK9XcON8WN3rU0m+oe7wP+obhfNk24LRG3DWj81UqwVSw4kNpSkWxcvXoxmCw/3GG6SUdSkmImNeHEi4kwkiQH6Lsq37RV1OBdAcOQEumFcMRQJ87W219u8uE1mYnVNLdEzQnNZSFiBWLYS2x+cYlZAzMG+Q+sbDjmCte5OTk5yDmn3PL1i0wUoNGXgSy1hStUh4S5RK9AhaguWWo9KvzUNB8CgUCJhug5F664GiL7hnCzdRc3AJuxtltYm3OqEndS0hFxW/zCrmHz6p1nZK3BjzTqAECMhGpeUPBETKpqe0TrAxCPcbrbZHKh7UbjpsWdSCmOcVfE4h/Slb+ruXoEbZuIQD8jCmhFRzhJ++m+uGwEWTQCk1nu4MZCo15qJYx9R10Y/1QMiDdkp5vJYXw5rsXgxJrNns83teiXjYBvTw4krx/OF6eCMEbViN4XFoMjWaQuO8rCO6TDRK7ym/s9DPdd9C6wdjNQY3YiROpB1jgdZFjhssa+I7DFnAGqZDhFpaQOxBVe3OYJMpNp1kEW11TNpa7bcdMsRZHXszJkzVgQhc9XAKIimR4NIQtJuSx1F3T3MiHuGmcDmaClmXs3OzjpHi3nWFQ0AWGU7eK2Jv8s6sF2bc1caejXEaLeh0kKScak5iArWSDkC2fTBPbe09CKwk++jjz7SXed9TE9PN+cz9RhuKNf8JAb/mWFmm97zWLSMRew8JjRjqBOD2HOAt8SeoO3xaDIXxBlQKqjW7ofYZ3wgnmlSkaJPOOb2a5L8egx/bzHg2wXcMaBJh8zvWJiEyjhx7gNZg2Z/0WhmwmL6lWWD/VBQ4jYK7k3UpBhI1HXZBKR8qJiEPQHOx4v5jiSr4FrSdXUYfL0fp/dmDnDQaXtwtNhxnKPAkj9n3QjqcME4xpOFLyJaB7rL804fcJ5CtyWXEKCGmNqrV69aaLufqBxatTqKRcR1iR9YtAUZWTuAyyHMnGrae1EJOhYmhiY6CO3JjbEPAYd0MmnTaQCNWyiIJJEx4Vz/rgrQgBTZJqCvI31zW8LRHqgsZHhdu3bNqw1tjzGas5krWh3svWYCUjZq1BV4dPQk4AgGQVaiTuy9dBdjtrlVBYgup2hvgyd6gTut21jYMM6eTZGFPrwI6DPHmB/suXTgEOLomF+5fVH53OK8OnqDpghSjCpmdg1asaczt2Qt4TF2hG9zim//Sku4mVuEZ0yliOoRE9APR2WvilaAJIdd2ySFkUGpi9ZVdixsUmgTComTH5OXtdbMK/oR8uhVsWGFZu3ruCLgIXCPnocRuobbUBpQXLIQgjUkRPwAYi2CsFgi2Zyse2SjyXkrO6t3Am5QJo6DLkhSQJSEEfRBg5t+WmWKOyHF/bBxjPvuMxN6DI6YBkBMEB3lmuRY51fsiuevVUEwqp9o/Y4heCDQOzrYjn3qotBGyFidsvrx4YcfqosawMTEhHiLE3xjls+AwG1+6KRWvhuQE06i9Pa5jztK3Vkj8dku4KhRORQCgQwU3Gy+Q94IWRGNnbfwFmtZO6nOFrodHun36h/Grt1vLExvY/G+rmiD2AuRi0thKOHrQLJlz9sRTAI8FEoNTqbF2sTzzkVve0Xr6EckbCfgxkDXGCKOzX5Xm6YmHCsARBF1iyvA6p1tFGLQTq+hLfM3K3beP2ptKgtbQACa+Jk3u+wxuE0p6H/iMCClYR88eBDcGfb9+/dZraQKsbHD6YpRpaEGdrrzFQ1fjBl4QB821i9ljY0pXme2y/ExXb16daDs3q5z8uyCiMWGJeiQIcfI/d0Abx9HmTHFRcL07p6IkR4TstGaxX6B6kdHR703joY0zSRU5jwqooxdZyldw80qdj68nX+m9zhUXNugz0Ut8Lj8NRm8xS454mRm0LplR6jecnCdZTQ+Pi71zsR+48aNHJqk3YovWXLsPep2WMQuUMPnrd2aEp3DOrEmdIJCRioseEV5iLbjnZDWgvjpiIRmLn6RgeampqZoC1PArzfnkO56mmPXcHsAkQtreNZDRH3mLTpB3yAcrNHyuj1fzmxmV7KmBM5w8uRJx52j+5+cL6jVzgN3gzUR7V6ObMfyPsG9BLf9gvZhxUWKp4IIlm0fm6ZMTOTLcVO3HdlMiW2/rLElIC4pv+swafQMoxFRQ/xMyN5jJlu0R+JqiHsD/LOjHdT42E9+b0q/7akEd4I7wb2t8v9k6Sr4lFL+WwAAAABJRU5ErkJggg=="}}]);