import React from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { useColorMode } from '@docusaurus/theme-common';
// import useThemeContext from '@theme/hooks/useThemeContext'
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";

import styles from "./index.module.css";
import bannerUrl from "@site/static/img/doc_banner.png";

function HomepageHeader() {
  // const { siteConfig } = useDocusaurusContext();
  // console.log(siteConfig)
  const {colorMode} = useColorMode();
  return (
    <header
      className={clsx(styles.heroBanner)}
      style={{
        paddingBottom:0,
        backgroundImage: `url(${bannerUrl})`,
        backgroundPosition:'center',
        backgroundSize:'cover',
        height:'600px',
        alignContent:'end',
        display:'flex'
      }}
    >
      <div 
        style={{
          alignSelf:'end',
          backgroundColor:colorMode === "dark" ? '#22222255' : '#dddddd55',
          padding:15,
          backdropFilter:'blur(5px)',
          WebkitBackdropFilter:'blur(5px)',
          width:'100vw'
        }}
      >
        <h1 className="hero__title">{"Lively v0.10.0 (beta)"}</h1>
        <p className="hero__subtitle">
          {
            "A highly configurable toolkit for commanding robots in mixed modalities"
          }
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            style={{
              margin:5,
              backgroundColor: '#ffffff55',
              backdropFilter: 'blur(5px)',
              WebkitBackdropFilter: 'blur(5px)'
            }}
            to="/docs/API/"
          >
            API
          </Link>
          <Link
            className="button button--secondary button--lg"
            style={{
              margin:5,
              backgroundColor: '#ffffff55',
              backdropFilter: 'blur(5px)',
              WebkitBackdropFilter: 'blur(5px)'
            }}
            to="/docs/Tutorials/"
          >
            Tutorials
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={"Lively"}
      description="A highly configurable toolkit for commanding robots in mixed modalities"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
