import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary')}>
      <div className="container text--center">
        <h2 className="hero__subtitle">{siteConfig.title}</h2>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={clsx('hero__buttons')}>
          <Link
            className="button button--secondary button--lg read-button"
            to="/docs/module-1/introduction-to-ros2-for-physical-ai">
            Read the Book
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features = [
    {
      title: 'Robotics Fundamentals',
      description: 'Learn the core principles of robotics, including kinematics, dynamics, and control systems.',
      icon: '🤖',
    },
    {
      title: 'AI & Machine Learning',
      description: 'Explore how artificial intelligence powers autonomous robotic systems and decision-making.',
      icon: '🧠',
    },
    {
      title: 'Humanoid Systems',
      description: 'Understand the design and control of bipedal robots and human-like mechanical systems.',
      icon: '🦾',
    },
    {
      title: 'Practical Applications',
      description: 'Discover real-world implementations of robotics in industry, healthcare, and research.',
      icon: '🔬',
    },
  ];

  return (
    <section className="features-section">
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div className="col col--3" key={idx}>
              <div className="feature-card text--center padding-horiz--md">
                <div className="feature-icon">{feature.icon}</div>
                <h3 className="feature-title">{feature.title}</h3>
                <p className="feature-description">{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}