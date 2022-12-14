import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Design Level',
    Svg: require('@site/static/img/design_level.svg').default,
    description: (
      <>
        The Design Level enables programming robots using a state-based approach.
      </>
    ),
  },
  {
    title: 'Develop Level',
    Svg: require('@site/static/img/develop_level.svg').default,
    description: (
      <>
        The Develop Level is configurable and portable, usable in applications such as ROS-based control and web-based simulation.
      </>
    ),
  },
  {
    title: 'Extend Level',
    Svg: require('@site/static/img/extend_level.svg').default,
    description: (
      <>
        The Extend Level supports the addition of new characteristics and goal specifications for greater customizability and extendability.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
