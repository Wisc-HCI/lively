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
  docs: [
    "API/index",
    {
      type: "category",
      label: "Solver",
      link: {
        type: "doc",
        id: "API/Solver/index",
      },
      items: [
        "API/Solver/initialization",
        {
          type: "category",
          label: "Methods",
          link: {
            type: "generated-index",
          },
          items: [
            "API/Solver/Methods/solve",
            "API/Solver/Methods/reset",
            "API/Solver/Methods/collision_normalization",
          ],
        },
        {
          type: "category",
          label: "Properties",
          link: {
            type: "doc",
            id: "API/Solver/Properties/index",
          },
          items: [],
        },
      ],
    },
    {
      type: "category",
      label: "Shapes",
      link: {
        type: "doc",
        id: "API/Shapes/index",
      },
      items: [
        "API/Shapes/box",
        "API/Shapes/sphere",
        "API/Shapes/cylinder",
        "API/Shapes/capsule",
      ],
    },
    "API/state",
    {
      type: "category",
      label: "Goals",
      link: {
        type: "doc",
        id: "API/Goals/goal",
      },
      items: [],
    },
    {
      type: "category",
      label: "Info",
      link: {
        type: "doc",
        id: "API/Info/index",
      },
      items: [
        "API/Info/transformInfo",
        "API/Info/mimicInfo",
        "API/Info/jointInfo",
        "API/Info/linkInfo",
        "API/Info/proximityInfo",
        "API/Info/collisionSettingInfo",
      ],
    },
    {
      type: "category",
      label: "Objectives",
      link: {
        type: "doc",
        id: "API/Objectives/index",
      },
      items: [
        "API/Objectives/base",
        "API/Objectives/bounding",
        "API/Objectives/matching",
        "API/Objectives/mirroring",
        "API/Objectives/liveliness",
        "API/Objectives/forces",
      ],
    },
    "API/collision",
  ],
  tutorials: [
    "Tutorials/index",
    "Tutorials/installation",
    {
      type: "category",
      label: "Basic Usage",
      link: {
        type: "generated-index",
      },
      items: [
        "Tutorials/initialization",
        "Tutorials/advanced_initialization",
        //"Tutorials/initialization_hook",
        "Tutorials/solving",
        "Tutorials/liveliness",
        "Tutorials/environment",
        "Tutorials/environment-sine",
      ],
    },
    {
      type: "category",
      label: "Customized Usage",
      link: {
        type: "generated-index",
      },
      items: ["Tutorials/adding_objectives", "Tutorials/adding_goals"],
    },
  ],
};

module.exports = sidebars;
