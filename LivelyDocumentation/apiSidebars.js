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
  
  tutorialSidebar: [
     'intro',
    {
      type: 'category',
      label: 'Goals',
      link: {
        type:'doc',
        id: 'Goals/goal'
      },
      items: [],    
    },
    {
      type: 'category',
      label: 'Objectives',
      link: {
        type:'doc',
        id: 'Objectives/objective'
      },
      items: [],
    },
    {
      type: 'category',
      label: 'Solver',
      link: {
        type:'doc',
        id: 'Solver/solver'
      },
      items:[
        {
          type: 'category',
          label: 'Methods',
          link: {
            type: 'generated-index',
          },
          items:[
            'Solver/Methods/solve',
            'Solver/Methods/solve_with_retries',
            'Solver/Methods/reset',
            'Solver/Methods/optimize',
            'Solver/Methods/get_current_state',
            'Solver/Methods/compute_average_distance_table',
        ]
        }
      ]
    }
  ],
   
};

module.exports = sidebars;
