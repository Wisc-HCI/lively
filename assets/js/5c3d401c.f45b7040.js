"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[2538],{3905:(t,e,n)=>{n.d(e,{Zo:()=>m,kt:()=>g});var a=n(67294);function r(t,e,n){return e in t?Object.defineProperty(t,e,{value:n,enumerable:!0,configurable:!0,writable:!0}):t[e]=n,t}function i(t,e){var n=Object.keys(t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(t);e&&(a=a.filter((function(e){return Object.getOwnPropertyDescriptor(t,e).enumerable}))),n.push.apply(n,a)}return n}function l(t){for(var e=1;e<arguments.length;e++){var n=null!=arguments[e]?arguments[e]:{};e%2?i(Object(n),!0).forEach((function(e){r(t,e,n[e])})):Object.getOwnPropertyDescriptors?Object.defineProperties(t,Object.getOwnPropertyDescriptors(n)):i(Object(n)).forEach((function(e){Object.defineProperty(t,e,Object.getOwnPropertyDescriptor(n,e))}))}return t}function o(t,e){if(null==t)return{};var n,a,r=function(t,e){if(null==t)return{};var n,a,r={},i=Object.keys(t);for(a=0;a<i.length;a++)n=i[a],e.indexOf(n)>=0||(r[n]=t[n]);return r}(t,e);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(t);for(a=0;a<i.length;a++)n=i[a],e.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(t,n)&&(r[n]=t[n])}return r}var p=a.createContext({}),d=function(t){var e=a.useContext(p),n=e;return t&&(n="function"==typeof t?t(e):l(l({},e),t)),n},m=function(t){var e=d(t.components);return a.createElement(p.Provider,{value:e},t.children)},u="mdxType",c={inlineCode:"code",wrapper:function(t){var e=t.children;return a.createElement(a.Fragment,{},e)}},s=a.forwardRef((function(t,e){var n=t.components,r=t.mdxType,i=t.originalType,p=t.parentName,m=o(t,["components","mdxType","originalType","parentName"]),u=d(n),s=r,g=u["".concat(p,".").concat(s)]||u[s]||c[s]||i;return n?a.createElement(g,l(l({ref:e},m),{},{components:n})):a.createElement(g,l({ref:e},m))}));function g(t,e){var n=arguments,r=e&&e.mdxType;if("string"==typeof t||r){var i=n.length,l=new Array(i);l[0]=s;var o={};for(var p in e)hasOwnProperty.call(e,p)&&(o[p]=e[p]);o.originalType=t,o[u]="string"==typeof t?t:r,l[1]=o;for(var d=2;d<i;d++)l[d]=n[d];return a.createElement.apply(null,l)}return a.createElement.apply(null,n)}s.displayName="MDXCreateElement"},16462:(t,e,n)=>{n.r(e),n.d(e,{assets:()=>p,contentTitle:()=>l,default:()=>c,frontMatter:()=>i,metadata:()=>o,toc:()=>d});var a=n(87462),r=(n(67294),n(3905));const i={},l="Bounding",o={unversionedId:"API/Objectives/bounding",id:"API/Objectives/bounding",title:"Bounding",description:"Bounding Objectives limit the space within which joints can assume angles and links can move or be oriented.",source:"@site/docs/API/Objectives/bounding.mdx",sourceDirName:"API/Objectives",slug:"/API/Objectives/bounding",permalink:"/lively/docs/API/Objectives/bounding",draft:!1,tags:[],version:"current",frontMatter:{},sidebar:"docs",previous:{title:"Base",permalink:"/lively/docs/API/Objectives/base"},next:{title:"Matching",permalink:"/lively/docs/API/Objectives/matching"}},p={},d=[{value:"Parameters for constructing Bounding Objectives",id:"parameters-for-constructing-bounding-objectives",level:2}],m={toc:d},u="wrapper";function c(t){let{components:e,...n}=t;return(0,r.kt)(u,(0,a.Z)({},m,n,{components:e,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"bounding"},"Bounding"),(0,r.kt)("p",null,"Bounding Objectives limit the space within which joints can assume angles and links can move or be oriented."),(0,r.kt)("table",null,(0,r.kt)("thead",{parentName:"table"},(0,r.kt)("tr",{parentName:"thead"},(0,r.kt)("th",{parentName:"tr",align:null},"Objective"),(0,r.kt)("th",{parentName:"tr",align:null},"Goals"),(0,r.kt)("th",{parentName:"tr",align:null},"Description"))),(0,r.kt)("tbody",{parentName:"table"},(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},"Position Bounding"),(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("a",{parentName:"td",href:"../Goals/goal"},(0,r.kt)("inlineCode",{parentName:"a"},"Ellipse"))),(0,r.kt)("td",{parentName:"tr",align:null},"Place the position of a link within a rotated ellipsoid")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},"Orientation Bounding"),(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("a",{parentName:"td",href:"../Goals/goal"},(0,r.kt)("inlineCode",{parentName:"a"},"RotationRange"))),(0,r.kt)("td",{parentName:"tr",align:null},"A region in orientation space with some delta around an orientation")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},"Joint Bounding"),(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("a",{parentName:"td",href:"../Goals/goal"},(0,r.kt)("inlineCode",{parentName:"a"},"JointBounding"))),(0,r.kt)("td",{parentName:"tr",align:null},"Limits the maximum and minimum values of a given joint")))),(0,r.kt)("h2",{id:"parameters-for-constructing-bounding-objectives"},"Parameters for constructing Bounding Objectives"),(0,r.kt)("table",null,(0,r.kt)("thead",{parentName:"table"},(0,r.kt)("tr",{parentName:"thead"},(0,r.kt)("th",{parentName:"tr",align:null},"Parameter"),(0,r.kt)("th",{parentName:"tr",align:null},"Objectives"),(0,r.kt)("th",{parentName:"tr",align:null},"Type"),(0,r.kt)("th",{parentName:"tr",align:null},"Optional"),(0,r.kt)("th",{parentName:"tr",align:null},"Defauly value/behavior"),(0,r.kt)("th",{parentName:"tr",align:null},"Description"))),(0,r.kt)("tbody",{parentName:"table"},(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"name")),(0,r.kt)("td",{parentName:"tr",align:null},"all of the objectives mentioned above"),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"none"),(0,r.kt)("td",{parentName:"tr",align:null},"Name can be arbitrary and should be unqiue")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"weight")),(0,r.kt)("td",{parentName:"tr",align:null},"all of the objectives mentioned above"),(0,r.kt)("td",{parentName:"tr",align:null},"float"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"none"),(0,r.kt)("td",{parentName:"tr",align:null},"Indicates the prioritization of the objective")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"link")),(0,r.kt)("td",{parentName:"tr",align:null},"all of the objectives mentioned above"),(0,r.kt)("td",{parentName:"tr",align:null},"string"),(0,r.kt)("td",{parentName:"tr",align:null},"no"),(0,r.kt)("td",{parentName:"tr",align:null},"none"),(0,r.kt)("td",{parentName:"tr",align:null},"The name of the link this objective is applied to")),(0,r.kt)("tr",{parentName:"tbody"},(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("inlineCode",{parentName:"td"},"goal")),(0,r.kt)("td",{parentName:"tr",align:null},"all of the objectives mentioned above"),(0,r.kt)("td",{parentName:"tr",align:null},(0,r.kt)("a",{parentName:"td",href:"../Goals/goal"},(0,r.kt)("inlineCode",{parentName:"a"},"Goal"))),(0,r.kt)("td",{parentName:"tr",align:null},"yes"),(0,r.kt)("td",{parentName:"tr",align:null},"origin for ",(0,r.kt)("inlineCode",{parentName:"td"},"Translation"),", no rotation for ",(0,r.kt)("inlineCode",{parentName:"td"},"Rotation"),", and 0 for both maximum and minimum for ",(0,r.kt)("inlineCode",{parentName:"td"},"JointBounding")),(0,r.kt)("td",{parentName:"tr",align:null},"The goal for objectives to achieve")))))}c.isMDXComponent=!0}}]);