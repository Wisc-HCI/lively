/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  
  sidebar: [
     'API/intro',
     'API/shapes',
     'API/state',
    {
      type: 'category',
      label: 'Goals',
      link: {
        type:'doc',
        id: 'API/Goals/goal'
      },
      items: [],    
    },
    {
      type: 'category',
      label: 'Objectives',
      link: {
        type:'doc',
        id: 'API/Objectives/objective'
      },
      items: [],
    },
    'API/collision',
    {
      type: 'category',
      label: 'Solver',
      link: {
        type:'doc',
        id: 'API/Solver/initialization'
      },
      items:[
         
        {
          type: 'category',
          label: 'Methods',
          link: {
            type: 'generated-index',
          },
          items:[
            'API/Solver/Methods/methods',
            'API/Solver/Methods/solve',
            'API/Solver/Methods/solve_with_retries',
            'API/Solver/Methods/reset',
            'API/Solver/Methods/optimize',
            'API/Solver/Methods/compute_average_distance_table',
        ]
        },
        {
          type: 'category',
          label: 'Properties',
          link:{
            type: 'doc',
            id: 'API/Solver/Properties/intro'
          },
          items:[
                ]
        },            
      ]
    },
    
    
  ],
   
};

module.exports = sidebars;
