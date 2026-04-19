import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import TeX from '@site/src/components/TeX';

import styles from '@site/src/pages/index.module.css';

interface Pillar {
  num: string;
  title: string;
  body: string;
  math: string;
}

interface Topic {
  num: string;
  topic: string;
  desc: string;
  status: 'live' | 'queued';
  statusLabel: string;
}

const PILLARS: ReadonlyArray<Pillar> = [
  {
    num: 'I',
    title: 'Explain it or you do not know it',
    body: 'Every concept must fit in two or three precise sentences — usable in interviews and when pairing with AI. Vague "roughly means…" does not count.',
    math: '\\text{clear definition} + \\text{physical meaning}',
  },
  {
    num: 'II',
    title: 'Situation-to-reasoning chain',
    body: 'Given X → pick tool Y → because principle Z → avoid pitfall W. Trains engineering judgment, not formula plug-and-chug.',
    math: 'X \\Rightarrow Y \\;\\text{(by } Z\\text{)}',
  },
  {
    num: 'III',
    title: 'Locate it in the loop',
    body: 'Every new term is placed in the sense → plan → act loop, so concepts never float detached from real robot systems.',
    math: '\\text{sense} \\to \\text{plan} \\to \\text{act}',
  },
];

const TOPICS: ReadonlyArray<Topic> = [
  {num: '01–03', topic: 'C++ Systems', desc: 'Memory management, multithreading, atomics, and lock-free programming.', status: 'live', statusLabel: 'LIVE'},
  {num: '04–06', topic: 'ROS 2', desc: 'Node communication, TF transforms, QoS, and executor design.', status: 'live', statusLabel: 'LIVE'},
  {num: '07–09', topic: 'Kinematics & Dynamics', desc: 'Forward/inverse kinematics, DH, Jacobian, singularities, Newton-Euler, Lagrangian.', status: 'live', statusLabel: 'LIVE'},
  {num: '10–12', topic: 'Planning & Avoidance', desc: 'A*/RRT path planning, trajectory optimization, dynamic obstacle avoidance.', status: 'live', statusLabel: 'LIVE'},
  {num: '13–16', topic: 'Control & Perception', desc: 'PID, impedance/admittance, MPC, visual servoing.', status: 'live', statusLabel: 'LIVE'},
  {num: '17', topic: 'SLAM', desc: 'Simultaneous localization and mapping in unknown environments.', status: 'live', statusLabel: 'LIVE'},
  {num: '18–20', topic: 'RL & Imitation', desc: 'MDP/Q-Learning → PPO/SAC → behavior cloning/DAgger.', status: 'live', statusLabel: 'LIVE'},
  {num: '21–22', topic: 'Sim-to-Real & LLMs', desc: 'Domain randomization, VLA, world models, embodied decision-making.', status: 'live', statusLabel: 'LIVE'},
];

function Masthead(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const today = new Date();
  const issue = `Vol. 01 · No. ${String(today.getFullYear()).slice(-2)}.${String(today.getMonth() + 1).padStart(2, '0')}`;

  return (
    <header className={styles.masthead}>
      <div className={styles.mastheadMark} aria-hidden="true">§</div>
      <div className={styles.mastheadInner}>
        <div>
          <div className={styles.kicker}>
            <span className={styles.kickerRule} />
            <span>{issue}</span>
            <span className={styles.kickerAccent}>— personal technical journal</span>
          </div>
          <h1 className={styles.title}>
            From derivation
            <br />
            <span className={styles.titleAccent}>to shippable</span>
            <br />
            robot systems.
          </h1>
          <p className={styles.lede}>
            {siteConfig.tagline}. Not a textbook, not a blog —
            a personal reference manual with progress tracking baked in.
          </p>
          <div className={styles.actions}>
            <Link className={styles.cta} to="/content/embodied-robotics/cpp-memory-optimization">
              Enter the first chapter
              <span className={styles.ctaArrow} aria-hidden="true">→</span>
            </Link>
            <Link className={styles.ctaGhost} to="/progress">
              View progress overview
            </Link>
          </div>
        </div>
        <aside className={styles.colophon} aria-label="Table of contents">
          <span className={styles.colophonLabel}>22 chapters · all live</span>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>01–03</span>
            <span className={styles.colophonName}>C++ Systems</span>
            <span className={styles.colophonVal}>live</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>04–06</span>
            <span className={styles.colophonName}>ROS 2</span>
            <span className={styles.colophonVal}>live</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>07–12</span>
            <span className={styles.colophonName}>Kinematics · Planning</span>
            <span className={styles.colophonVal}>live</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>13–17</span>
            <span className={styles.colophonName}>Control · Perception · SLAM</span>
            <span className={styles.colophonVal}>live</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>18–22</span>
            <span className={styles.colophonName}>RL · IL · Sim2Real · VLA</span>
            <span className={styles.colophonVal}>live</span>
          </div>
        </aside>
      </div>
    </header>
  );
}

function Manifesto(): ReactNode {
  return (
    <section className={styles.manifesto}>
      <div className={styles.manifestoInner}>
        <div className={styles.sectionLabel}>
          <span>Editorial principles</span>
        </div>
        <div className={styles.manifestoGrid}>
          {PILLARS.map((p) => (
            <article key={p.num} className={styles.manifestoItem}>
              <span className={styles.itemNum}>PRINCIPLE {p.num}</span>
              <h3 className={styles.itemTitle}>{p.title}</h3>
              <p className={styles.itemBody}>{p.body}</p>
              <div className={styles.itemMath}><TeX>{p.math}</TeX></div>
            </article>
          ))}
        </div>
      </div>
    </section>
  );
}

function Scope(): ReactNode {
  return (
    <section className={styles.scope}>
      <div className={styles.scopeInner}>
        <div className={styles.scopeIntro}>
          <div className={styles.sectionLabel} style={{marginBottom: '1.5rem'}}>
            <span>Current index</span>
          </div>
          <h2>Enter any book from here.</h2>
          <p>
            Each row is a body of material being absorbed. Chapters marked LIVE are ready;
            the rest are queued. Progress and timestamps live in this device&apos;s localStorage.
          </p>
        </div>
        <ol className={styles.scopeList}>
          {TOPICS.map((t) => (
            <li key={t.num} className={styles.scopeRow}>
              <span className={styles.scopeNum}>{t.num}</span>
              <span className={styles.scopeTopic}>{t.topic}</span>
              <span className={styles.scopeDesc}>{t.desc}</span>
              <span
                className={`${styles.scopeStatus} ${
                  t.status === 'live' ? styles.scopeStatusLive : ''
                }`}>
                {t.statusLabel}
              </span>
            </li>
          ))}
        </ol>
      </div>
    </section>
  );
}

function Method(): ReactNode {
  return (
    <section className={styles.method}>
      <div className={styles.methodInner}>
        <p className={styles.methodQuote}>
          After finishing a chapter you should be able to re-derive it on a whiteboard,
          sketch the intuition, and explain which signal it represents in a real system.
        </p>
        <div className={styles.methodAttribution}>
          The delivery standard for this manual
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={siteConfig.title} description={siteConfig.tagline}>
      <div className={styles.page}>
        <Masthead />
        <Manifesto />
        <Scope />
        <Method />
      </div>
    </Layout>
  );
}
