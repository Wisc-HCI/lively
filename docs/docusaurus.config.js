// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion
const meshLoader = require("./plugins/mesh-loader");
const lightCodeTheme = require("prism-react-renderer/themes/github");
const darkCodeTheme = require("prism-react-renderer/themes/dracula");

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Lively",
  tagline: "A highly configurable toolkit for commanding robots in mixed modalities",
  url: "https://wisc-hci.github.io",
  baseUrl: "/lively/",
  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",
  favicon: "/img/favicon.ico",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "Wisc-HCI", // Usually your GitHub org/user name.
  projectName: "lively", // Usually your repo name.
  deploymentBranch: "gh_pages",
  //trailingSlash : false,

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  markdown: {
    mermaid: true,
  },

  presets: [
    [
      "classic",
      {
        docs: {
          path: "docs",
          sidebarPath: require.resolve("./sidebars.js"),
        },
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      },
    ],
  ],

  themes: ["@docusaurus/theme-mermaid",'@docusaurus/theme-live-codeblock'],
  plugins: ['./plugins/mesh-loader'], // loader required for .glb
  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: "Lively",
        logo: {
          alt: "Lively Logo",
          src: "img/logo.svg",
        },
        items: [
          {
            type: "doc",
            docId: "API/index",
            position: "left",
            label: "API",
          },
          {
            type: "doc",
            docId: "Tutorials/index",
            position: "left",
            label: "Tutorials",
          },
          {
            href: "https://github.com/Wisc-HCI/lively",
            label: "GitHub",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        links: [
          {
            title: "Docs",
            items: [
              {
                label: "API",
                to: "/docs/API/",
              },
            ],
          },

          {
            title: "More",
            items: [
              {
                label: "Tutorial",
                to: "/docs/Tutorials/",
              },
              {
                label: "GitHub",
                href: "https://github.com/Wisc-HCI/lively.git",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Lively.`,
      },
      prism: {
        additionalLanguages: ["rust"],
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
      mermaid: {
        theme: { light: "default", dark: "dark" },
      },
    }),
};

module.exports = config;
