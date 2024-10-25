import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Beyond Boundaries: Collaborative Robots for the New Frontier',
    imgSrc: require('@site/static/img/homepage_1.webp').default,
    description: (
      <>
        Designing autonomous robots to navigate and unveil the mysteries of distant worlds.
      </>
    ),
  },
  {
    title: 'Redefining Exploration with Smart, Connected Robotics',
    imgSrc: require('@site/static/img/homepage_2.webp').default,
    description: (
      <>
        Pioneering a new era of exploration with networks of autonomous robots that work together across vast distances.
      </>
    ),
  },
  {
    title: 'Unleashing Intelligent Swarms Beyond Earth',
    imgSrc: require('@site/static/img/homepage_3.webp').default,
    description: (
      <>
       Building the foundation for intelligent, interconnected robotic systems that redefine exploration beyond Earth.
      </>
    ),
  },
];

function Feature({imgSrc, title, description}) {
  return (
    <div className={clsx('col col--4',styles.featureColumn)}>
      <div className="text--center">
        <img src={imgSrc} alt={title} className={styles.featureImg} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={clsx("row", styles.featuresContainer)}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
