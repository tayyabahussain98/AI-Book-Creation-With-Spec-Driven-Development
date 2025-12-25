import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

// Module data for the 4 main modules
const modules = [
  {
    title: 'Module 1: ROS 2 Robotic Nervous System',
    description: 'Master ROS 2 fundamentals, nodes, topics, services, and actions for robot communication.',
    icon: 'robot',
    link: '/docs/module-1-ros2/chapter-1-intro-physical-ai'
  },
  {
    title: 'Module 2: Digital Twin Simulation',
    description: 'Build realistic robot simulations using Gazebo and create digital twin environments.',
    icon: 'cube',
    link: '/docs/module-2-digital-twin/chapter-1-foundations'
  },
  {
    title: 'Module 3: AI-Robot Brain',
    description: 'Integrate NVIDIA Isaac for perception, planning, and control in robotic systems.',
    icon: 'brain',
    link: '/docs/module-3-isaac-ai/chapter-1-isaac-overview'
  },
  {
    title: 'Module 4: Vision-Language-Action',
    description: 'Build VLA systems that understand vision, language, and generate robot actions.',
    icon: 'eye',
    link: '/docs/module-4-vla/chapter-1-vla-foundations'
  }
];

// SVG Icons as simple components
const RobotIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
    <rect x="4" y="4" width="16" height="16" rx="2"/>
    <rect x="9" y="9" width="6" height="6"/>
    <line x1="9" y1="2" x2="9" y2="4"/>
    <line x1="15" y1="2" x2="15" y2="4"/>
    <line x1="9" y1="20" x2="9" y2="22"/>
    <line x1="15" y1="20" x2="15" y2="22"/>
    <line x1="20" y1="9" x2="22" y2="9"/>
    <line x1="20" y1="15" x2="22" y2="15"/>
    <line x1="2" y1="9" x2="4" y2="9"/>
    <line x1="2" y1="15" x2="4" y2="15"/>
  </svg>
);

const CubeIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
    <path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/>
    <polyline points="3.27 6.96 12 12.01 20.73 6.96"/>
    <line x1="12" y1="22.08" x2="12" y2="12"/>
  </svg>
);

const BrainIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
    <path d="M9.5 2A2.5 2.5 0 0 1 12 4.5v15a2.5 2.5 0 0 1-4.96.44 2.5 2.5 0 0 1-2.96-3.08 3 3 0 0 1-.34-5.58 2.5 2.5 0 0 1 1.32-4.24 2.5 2.5 0 0 1 1.98-3A2.5 2.5 0 0 1 9.5 2Z"/>
    <path d="M14.5 2A2.5 2.5 0 0 0 12 4.5v15a2.5 2.5 0 0 0 4.96.44 2.5 2.5 0 0 0 2.96-3.08 3 3 0 0 0 .34-5.58 2.5 2.5 0 0 0-1.32-4.24 2.5 2.5 0 0 0-1.98-3A2.5 2.5 0 0 0 14.5 2Z"/>
  </svg>
);

const EyeIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
    <path d="M2 12s3-7 10-7 10 7 10 7-3 7-10 7-10-7-10-7Z"/>
    <circle cx="12" cy="12" r="3"/>
  </svg>
);

const iconComponents: Record<string, () => ReactNode> = {
  robot: RobotIcon,
  cube: CubeIcon,
  brain: BrainIcon,
  eye: EyeIcon
};

function HeroSection() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <section className={clsx('hero', styles.hero)}>
      <div className={styles.heroContent}>
        <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
        <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
        <div className={styles.heroActions}>
          <Link to="/docs/module-1-ros2/chapter-1-intro-physical-ai" className={styles.heroCta}>
            Start Reading
          </Link>
          <a href="#modules" className={styles.heroSecondary}>
            View Modules
          </a>
        </div>
      </div>
    </section>
  );
}

function ModuleCard({module}: {module: typeof modules[0]}) {
  const IconComponent = iconComponents[module.icon] || RobotIcon;

  return (
    <Link to={module.link} className={styles.moduleCard}>
      <div className={styles.moduleCardIcon}>
        <IconComponent />
      </div>
      <h3 className={styles.moduleCardTitle}>{module.title}</h3>
      <p className={styles.moduleCardDescription}>{module.description}</p>
      <span className={styles.moduleCardLink}>
        Start Learning <span className={styles.arrow}>→</span>
      </span>
    </Link>
  );
}

function ModuleCardsSection() {
  return (
    <section id="modules" className={styles.modulesSection}>
      <h2 className={styles.modulesSectionTitle}>Book Modules</h2>
      <div className={styles.moduleCards}>
        {modules.map((module, index) => (
          <ModuleCard key={index} module={module} />
        ))}
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title} - Modern Robotics Education`}
      description="Master ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action systems for physical AI robotics">
      <HeroSection />
      <main>
        <ModuleCardsSection />
      </main>
    </Layout>
  );
}
