import React from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";

import styles from "./index.module.css";
import bannerUrl from "@site/static/img/doc_banner.png";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header
      className={clsx("hero hero--primary", styles.heroBanner)}
      style={{
        backgroundImage: `url(${bannerUrl})`,
        backgroundPosition:'center',
        backgroundSize:'cover'
      }}
    >
      <div 
        className="container" 
        style={{
          backgroundColor:'#99999955',
          padding:15,
          borderRadius:4,
          backdropFilter:'blur(5px)',
          WebkitBackdropFilter:'blur(5px)'
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
            to="/docs/API/intro"
          >
            Lively API
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
      title={""}
      description="A highly configurable toolkit for commanding robots in mixed modalities"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
